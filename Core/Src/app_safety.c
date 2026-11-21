/**
  ******************************************************************************
  * @file    app_safety.c
  * @brief   Achizitie senzori, protectii (scurtcircuit / suprasarcina / incendiu)
  *          si protocol UART cu ESP32. Complet non-blocant (HAL_GetTick()).
  *
  *  Fluxul principal (App_Safety_Process, apelat continuu din while(1)):
  *    1. la fiecare 1 ms: citeste adc_dma_buf[] -> filtre medie mobila (64)
  *    2. CALIBRATING: 200 esantioane cu releul decuplat -> offset zero ACS712
  *       RUNNING    : protectii + iesiri (buzzer, lumina intermitenta)
  *    3. proceseaza comenzile primite pe UART (linie terminata cu '\n')
  *    4. trimite DATA,... la 500 ms sau imediat la schimbarea starii
  *
  *  Note de proiectare:
  *   - DMA ruleaza circular; intreruperile HT/TC ale DMA sunt dezactivate dupa
  *     pornire (ADC continuu ~58 kHz/secventa ar genera ~117k IRQ/s inutile).
  *     Bufferul este citit periodic din bucla principala.
  *   - Scurtcircuitul se evalueaza pe o fereastra scurta (4 ms), nu pe media
  *     de 64 esantioane, pentru reactie rapida dar imuna la un singur varf.
  *   - Avariile se memoreaza (latch) pana la CMD,RESET_ALARM.
  *     Prioritate cod raportat: SCURTCIRCUIT(2) > INCENDIU(3) > SUPRASARCINA(1).
  ******************************************************************************
  */
#include "app_safety.h"
#include "adc.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ============================ Definitii interne ============================ */
#define FAULT_BIT_OVERLOAD   (1U << 0)
#define FAULT_BIT_SHORT      (1U << 1)
#define FAULT_BIT_FIRE       (1U << 2)

typedef enum
{
  APP_STATE_CALIBRATING = 0,
  APP_STATE_RUNNING
} AppState_t;

typedef struct
{
  uint16_t buf[FILTER_LEN];
  uint32_t sum;
  uint16_t idx;
  uint16_t count;
} MovAvg_t;

/* ============================ Date ========================================= */
volatile uint16_t adc_dma_buf[ADC_CH_COUNT];

static MovAvg_t  s_filter[ADC_CH_COUNT];
static uint16_t  s_i_fast_buf[CURRENT_FAST_LEN];
static uint32_t  s_i_fast_sum;
static uint8_t   s_i_fast_idx;
static uint8_t   s_i_fast_count;

static AppState_t s_state;
static uint32_t   s_init_tick;
static uint32_t   s_cal_sum;
static uint16_t   s_cal_count;
static float      s_acs_zero_v = ACS_ZERO_NOMINAL_V;
static uint8_t    s_calib_ok;

static float      s_i_overload_max = I_OVERLOAD_DEFAULT;

/* Masuratori (valori filtrate) */
static float s_voltage_v;
static float s_current_a;       /* medie 64 esantioane */
static float s_current_fast_a;  /* medie 4 esantioane  */
static float s_temp_c;
static float s_smoke_pct;

/* Stare protectii / iesiri */
static uint8_t     s_fault_flags;
static FaultCode_t s_fault_code_sent = FAULT_NONE;
static uint8_t     s_relay_on;
static uint8_t     s_light_on;
static uint8_t     s_buzzer_on;

static uint8_t  s_overload_timing;
static uint32_t s_overload_start_tick;
static uint8_t  s_fire_timing;
static uint32_t s_fire_start_tick;
static uint32_t s_blink_tick;

static uint32_t s_last_sample_tick;
static uint32_t s_last_tx_tick;
static uint8_t  s_tx_request;           /* trimitere imediata ceruta */

/* UART RX (completat in ISR) */
static uint8_t           s_rx_byte;
static char              s_rx_line[UART_RX_LINE_MAX];
static uint16_t          s_rx_len;
static uint8_t           s_rx_discard;  /* linie prea lunga / eroare -> ignorata */
static char              s_cmd_line[UART_RX_LINE_MAX];
static volatile uint8_t  s_cmd_ready;
static volatile uint8_t  s_rx_rearm;

/* UART TX */
static char s_tx_buf[UART_TX_BUF_LEN];

/* ============================ Prototipuri ================================== */
static void        adc_start(void);
static void        adc_check_overrun(void);
static void        sample_inputs(uint32_t now);
static void        compute_measurements(void);
static void        run_protections(uint32_t now);
static void        update_outputs(uint32_t now);
static void        raise_fault(uint8_t bit, uint32_t now);
static void        clear_all_faults(void);
static FaultCode_t fault_code(void);
static void        process_command(char *line);
static void        telemetry_task(uint32_t now);
static void        uart_rx_arm(void);

static void relay_set(uint8_t on);
static void light_set(uint8_t on);
static void buzzer_set(uint8_t on);

/* ============================ Utilitare ==================================== */
static inline float adc_to_volt(float raw)
{
  return (raw / ADC_FULL_SCALE) * ADC_VREF_V;
}

static inline float clampf(float v, float lo, float hi)
{
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static void movavg_push(MovAvg_t *f, uint16_t v)
{
  if (f->count == FILTER_LEN)
  {
    f->sum -= f->buf[f->idx];
  }
  else
  {
    f->count++;
  }
  f->buf[f->idx] = v;
  f->sum += v;
  f->idx = (uint16_t)((f->idx + 1U) % FILTER_LEN);
}

static float movavg_mean(const MovAvg_t *f)
{
  return (f->count != 0U) ? ((float)f->sum / (float)f->count) : 0.0f;
}

static void pin_write(GPIO_TypeDef *port, uint16_t pin, uint8_t on, uint8_t active_low)
{
  GPIO_PinState level = (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  if (active_low != 0U)
  {
    level = (level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }
  HAL_GPIO_WritePin(port, pin, level);
}

static void relay_set(uint8_t on)
{
  on = (on != 0U) ? 1U : 0U;
  pin_write(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, on, RELAY_ACTIVE_LOW);
  if (s_relay_on != on)
  {
    s_relay_on = on;
    s_tx_request = 1U;
  }
}

static void light_set(uint8_t on)
{
  s_light_on = (on != 0U) ? 1U : 0U;
  pin_write(RELAY_LIGHT_GPIO_Port, RELAY_LIGHT_Pin, s_light_on, LIGHT_ACTIVE_LOW);
}

static void buzzer_set(uint8_t on)
{
  s_buzzer_on = (on != 0U) ? 1U : 0U;
  pin_write(BUZZER_GPIO_Port, BUZZER_Pin, s_buzzer_on, (BUZZER_ACTIVE_HIGH != 0) ? 0U : 1U);
}

/* ============================ API public =================================== */
void App_Safety_Init(void)
{
  /* 1. Stare sigura a iesirilor: tot decuplat */
  s_relay_on = 1U;       /* forteaza scrierea pinului in relay_set() */
  relay_set(0U);
  light_set(0U);
  buzzer_set(0U);

  /* 2. Reset filtre si stare */
  memset(s_filter, 0, sizeof(s_filter));
  memset(s_i_fast_buf, 0, sizeof(s_i_fast_buf));
  s_i_fast_sum   = 0U;
  s_i_fast_idx   = 0U;
  s_i_fast_count = 0U;

  s_fault_flags     = 0U;
  s_fault_code_sent = FAULT_NONE;
  s_overload_timing = 0U;
  s_fire_timing     = 0U;
  s_i_overload_max  = I_OVERLOAD_DEFAULT;

  s_state       = APP_STATE_CALIBRATING;
  s_cal_sum     = 0U;
  s_cal_count   = 0U;
  s_acs_zero_v  = ACS_ZERO_NOMINAL_V;
  s_calib_ok    = 0U;

  s_init_tick        = HAL_GetTick();
  s_last_sample_tick = s_init_tick;
  s_last_tx_tick     = s_init_tick;
  s_blink_tick       = s_init_tick;
  s_tx_request       = 0U;

  /* 3. Pornire ADC + DMA circular */
  adc_start();

  /* 4. Receptie UART pe intrerupere, cate 1 byte */
  s_rx_len     = 0U;
  s_rx_discard = 0U;
  s_cmd_ready  = 0U;
  uart_rx_arm();
}

void App_Safety_Process(void)
{
  uint32_t now = HAL_GetTick();

  adc_check_overrun();

  /* Re-armare receptie UART daca a esuat in ISR */
  if (s_rx_rearm != 0U)
  {
    HAL_NVIC_DisableIRQ(USART6_IRQn);
    uart_rx_arm();
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  }

  /* Esantionare + protectii la fiecare SAMPLE_PERIOD_MS */
  if ((now - s_last_sample_tick) >= SAMPLE_PERIOD_MS)
  {
    s_last_sample_tick = now;
    sample_inputs(now);
    compute_measurements();

    if (s_state == APP_STATE_RUNNING)
    {
      run_protections(now);
    }
  }

  update_outputs(now);

  /* Comanda completa primita pe UART */
  if (s_cmd_ready != 0U)
  {
    char line[UART_RX_LINE_MAX];
    __DMB();
    memcpy(line, s_cmd_line, sizeof(line));
    line[UART_RX_LINE_MAX - 1U] = '\0';
    s_cmd_ready = 0U;
    process_command(line);
  }

  telemetry_task(now);
}

void App_Safety_GetStatus(SafetyStatus_t *out)
{
  if (out == NULL)
  {
    return;
  }
  out->voltage_v      = s_voltage_v;
  out->current_a      = s_current_a;
  out->temp_c         = s_temp_c;
  out->smoke_pct      = s_smoke_pct;
  out->fault          = fault_code();
  out->relay_on       = s_relay_on;
  out->calibrated     = (s_state == APP_STATE_RUNNING) ? 1U : 0U;
  out->calib_ok       = s_calib_ok;
  out->acs_zero_v     = s_acs_zero_v;
  out->i_overload_max = s_i_overload_max;
}

/* ============================ ADC ========================================== */
static void adc_start(void)
{
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(uintptr_t)adc_dma_buf, ADC_CH_COUNT) != HAL_OK)
  {
    Error_Handler();
  }
  /* Bufferul e citit prin polling: nu avem nevoie de IRQ la half/full transfer */
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC);
}

static void adc_check_overrun(void)
{
  /* Overrun (nu ar trebui sa apara cu DMA circular) -> repornire ADC */
  if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR))
  {
    (void)HAL_ADC_Stop_DMA(&hadc1);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
    adc_start();
  }
}

static void sample_inputs(uint32_t now)
{
  uint16_t raw[ADC_CH_COUNT];
  uint32_t ch;

  for (ch = 0U; ch < ADC_CH_COUNT; ch++)
  {
    raw[ch] = adc_dma_buf[ch];
    movavg_push(&s_filter[ch], raw[ch]);
  }

  /* Fereastra scurta pentru curent (detectie scurtcircuit) */
  if (s_i_fast_count == CURRENT_FAST_LEN)
  {
    s_i_fast_sum -= s_i_fast_buf[s_i_fast_idx];
  }
  else
  {
    s_i_fast_count++;
  }
  s_i_fast_buf[s_i_fast_idx] = raw[ADC_IDX_CURRENT];
  s_i_fast_sum += raw[ADC_IDX_CURRENT];
  s_i_fast_idx = (uint8_t)((s_i_fast_idx + 1U) % CURRENT_FAST_LEN);

  /* Auto-calibrare offset zero ACS712 (releul este decuplat) */
  if (s_state == APP_STATE_CALIBRATING)
  {
    if ((now - s_init_tick) < CAL_SETTLE_MS)
    {
      return;
    }

    s_cal_sum += raw[ADC_IDX_CURRENT];
    s_cal_count++;

    if (s_cal_count >= CAL_SAMPLES)
    {
      float zero_v = adc_to_volt((float)s_cal_sum / (float)s_cal_count);

      if ((zero_v >= ACS_ZERO_MIN_V) && (zero_v <= ACS_ZERO_MAX_V))
      {
        s_acs_zero_v = zero_v;
        s_calib_ok   = 1U;
      }
      else
      {
        /* Senzor deconectat / alimentare gresita: folosim valoarea nominala */
        s_acs_zero_v = ACS_ZERO_NOMINAL_V;
        s_calib_ok   = 0U;
      }

      s_state = APP_STATE_RUNNING;
      relay_set(1U);                 /* anclansare automata dupa calibrare */
      s_tx_request = 1U;
    }
  }
}

static void compute_measurements(void)
{
  float v_volt  = adc_to_volt(movavg_mean(&s_filter[ADC_IDX_VOLTAGE]));
  float v_curr  = adc_to_volt(movavg_mean(&s_filter[ADC_IDX_CURRENT]));
  float v_smoke = adc_to_volt(movavg_mean(&s_filter[ADC_IDX_SMOKE]));
  float v_temp  = adc_to_volt(movavg_mean(&s_filter[ADC_IDX_TEMP]));
  float v_fast  = (s_i_fast_count != 0U)
                  ? adc_to_volt((float)s_i_fast_sum / (float)s_i_fast_count)
                  : s_acs_zero_v;

  s_voltage_v      = v_volt * VOLT_DIV_FACTOR;
  s_current_a      = (v_curr - s_acs_zero_v) / ACS_SENS_V_PER_A;
  s_current_fast_a = (v_fast - s_acs_zero_v) / ACS_SENS_V_PER_A;
  s_smoke_pct      = clampf((v_smoke / MQ2_FULL_SCALE_V) * 100.0f, 0.0f, 100.0f);
  s_temp_c         = v_temp * LM35_DEG_PER_V;
}

/* ============================ Protectii ==================================== */
static void run_protections(uint32_t now)
{
  /* --- 1. Scurtcircuit: instantaneu, pe fereastra scurta ------------------ */
  if (((s_fault_flags & FAULT_BIT_SHORT) == 0U) &&
      (fabsf(s_current_fast_a) > I_SHORT_MAX))
  {
    raise_fault(FAULT_BIT_SHORT, now);
  }

  /* --- 2. Suprasarcina: > i_overload_max timp de 2000 ms consecutive ------- */
  if ((s_fault_flags & (FAULT_BIT_SHORT | FAULT_BIT_OVERLOAD)) == 0U)
  {
    if (fabsf(s_current_a) > s_i_overload_max)
    {
      if (s_overload_timing == 0U)
      {
        s_overload_timing     = 1U;
        s_overload_start_tick = now;
      }
      else if ((now - s_overload_start_tick) >= OVERLOAD_TRIP_MS)
      {
        s_overload_timing = 0U;
        raise_fault(FAULT_BIT_OVERLOAD, now);
      }
    }
    else
    {
      s_overload_timing = 0U;        /* a scazut sub prag -> reset temporizare */
    }
  }
  else
  {
    s_overload_timing = 0U;
  }

  /* --- 3. Incendiu: fum > 50 % SAU temperatura > 30 C --------------------- */
  if ((s_fault_flags & FAULT_BIT_FIRE) == 0U)
  {
    uint8_t smoke_enabled = ((now - s_init_tick) >= MQ2_WARMUP_MS) ? 1U : 0U;
    uint8_t fire_cond = ((smoke_enabled && (s_smoke_pct > SMOKE_ALARM_PCT)) ||
                         (s_temp_c > TEMP_ALARM_C)) ? 1U : 0U;

    if (fire_cond != 0U)
    {
      if (s_fire_timing == 0U)
      {
        s_fire_timing     = 1U;
        s_fire_start_tick = now;
      }
      if ((now - s_fire_start_tick) >= FIRE_CONFIRM_MS)
      {
        s_fire_timing = 0U;
        raise_fault(FAULT_BIT_FIRE, now);
      }
    }
    else
    {
      s_fire_timing = 0U;
    }
  }
}

static void raise_fault(uint8_t bit, uint32_t now)
{
  s_fault_flags |= bit;

  if ((bit & (FAULT_BIT_SHORT | FAULT_BIT_OVERLOAD)) != 0U)
  {
    relay_set(0U);                   /* decuplare imediata PG2 */
  }
  if ((bit & FAULT_BIT_FIRE) != 0U)
  {
    light_set(1U);                   /* porneste lumina intermitenta */
    s_blink_tick = now;
  }
  buzzer_set(1U);
  s_tx_request = 1U;
}

static void clear_all_faults(void)
{
  s_fault_flags     = 0U;
  s_overload_timing = 0U;
  s_fire_timing     = 0U;
  buzzer_set(0U);
  light_set(0U);
  relay_set(1U);                     /* recuplare PG2 */
  s_tx_request = 1U;
}

static FaultCode_t fault_code(void)
{
  if ((s_fault_flags & FAULT_BIT_SHORT) != 0U)    { return FAULT_SHORT_CIRCUIT; }
  if ((s_fault_flags & FAULT_BIT_FIRE) != 0U)     { return FAULT_FIRE; }
  if ((s_fault_flags & FAULT_BIT_OVERLOAD) != 0U) { return FAULT_OVERLOAD; }
  return FAULT_NONE;
}

static void update_outputs(uint32_t now)
{
  /* Buzzer activ cat timp exista orice avarie memorata */
  uint8_t want_buzzer = (s_fault_flags != 0U) ? 1U : 0U;
  if (want_buzzer != s_buzzer_on)
  {
    buzzer_set(want_buzzer);
  }

  /* Lumina de avarie intermitenta (300 ms) la incendiu */
  if ((s_fault_flags & FAULT_BIT_FIRE) != 0U)
  {
    if ((now - s_blink_tick) >= FIRE_BLINK_MS)
    {
      s_blink_tick = now;
      light_set((s_light_on != 0U) ? 0U : 1U);
    }
  }
  else if (s_light_on != 0U)
  {
    light_set(0U);
  }

  /* Orice schimbare a codului de avarie -> trimitere imediata */
  if (fault_code() != s_fault_code_sent)
  {
    s_tx_request = 1U;
  }
}

/* ============================ Comenzi UART ================================= */
static void process_command(char *line)
{
  size_t len = strlen(line);

  /* elimina spatii / CR de la final */
  while ((len > 0U) && ((line[len - 1U] == ' ') || (line[len - 1U] == '\r') ||
                        (line[len - 1U] == '\t')))
  {
    line[--len] = '\0';
  }
  if (len == 0U)
  {
    return;
  }

  if (strcmp(line, "CMD,RESET_ALARM") == 0)
  {
    if (s_state == APP_STATE_RUNNING)
    {
      clear_all_faults();
    }
    s_tx_request = 1U;
  }
  else if (strncmp(line, "CMD,RELAY,", 10) == 0)
  {
    const char *arg = &line[10];

    /* Comanda manuala permisa doar fara avarie si dupa calibrare */
    if ((s_state == APP_STATE_RUNNING) && (fault_code() == FAULT_NONE))
    {
      if (strcmp(arg, "1") == 0)
      {
        relay_set(1U);
      }
      else if (strcmp(arg, "0") == 0)
      {
        relay_set(0U);
        s_overload_timing = 0U;
      }
    }
    s_tx_request = 1U;              /* ESP32 vede starea reala a releului */
  }
  else if (strncmp(line, "CMD,SET_IMAX,", 13) == 0)
  {
    const char *arg = &line[13];
    char *end = NULL;
    float val = strtof(arg, &end);

    if ((end != arg) && (*end == '\0') && isfinite(val) &&
        (val >= I_OVERLOAD_MIN) && (val < I_SHORT_MAX))
    {
      s_i_overload_max  = val;
      s_overload_timing = 0U;
    }
    s_tx_request = 1U;
  }
  /* comenzile necunoscute sunt ignorate */
}

/* ============================ Telemetrie =================================== */
static void telemetry_task(uint32_t now)
{
  if ((s_tx_request == 0U) && ((now - s_last_tx_tick) < TELEMETRY_PERIOD_MS))
  {
    return;
  }
  if (huart6.gState != HAL_UART_STATE_READY)
  {
    s_tx_request = 1U;              /* transmisia anterioara inca ruleaza */
    return;
  }

  FaultCode_t code = fault_code();
  float i_disp = (fabsf(s_current_a) < CURRENT_DISPLAY_DEADBAND_A) ? 0.0f : s_current_a;
  unsigned smoke = (unsigned)(s_smoke_pct + 0.5f);

  int n = snprintf(s_tx_buf, sizeof(s_tx_buf), "DATA,%.2f,%.2f,%.1f,%u,%d,%u\n",
                   (double)s_voltage_v, (double)i_disp, (double)s_temp_c,
                   smoke, (int)code, (unsigned)s_relay_on);

  if ((n > 0) && ((size_t)n < sizeof(s_tx_buf)))
  {
    if (HAL_UART_Transmit_IT(&huart6, (uint8_t *)s_tx_buf, (uint16_t)n) == HAL_OK)
    {
      s_fault_code_sent = code;
      s_last_tx_tick    = now;
      s_tx_request      = 0U;
    }
  }
}

/* ============================ Callback-uri HAL UART ======================== */
static void uart_rx_arm(void)
{
  if (huart6.RxState == HAL_UART_STATE_READY)
  {
    if (HAL_UART_Receive_IT(&huart6, &s_rx_byte, 1U) != HAL_OK)
    {
      s_rx_rearm = 1U;
      return;
    }
  }
  s_rx_rearm = 0U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART6)
  {
    return;
  }

  char c = (char)s_rx_byte;

  if (c == '\n')
  {
    if ((s_rx_discard == 0U) && (s_rx_len > 0U) && (s_cmd_ready == 0U))
    {
      memcpy(s_cmd_line, s_rx_line, s_rx_len);
      s_cmd_line[s_rx_len] = '\0';
      __DMB();
      s_cmd_ready = 1U;
    }
    /* daca o comanda anterioara nu a fost inca procesata, linia noua se pierde */
    s_rx_len     = 0U;
    s_rx_discard = 0U;
  }
  else if (c == '\r')
  {
    /* ignorat */
  }
  else if (s_rx_len < (UART_RX_LINE_MAX - 1U))
  {
    s_rx_line[s_rx_len++] = c;
  }
  else
  {
    s_rx_discard = 1U;              /* linie prea lunga */
  }

  uart_rx_arm();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART6)
  {
    return;
  }
  /* ORE/FE/NE: linia curenta e corupta -> o aruncam si re-armam receptia */
  if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
  }
  s_rx_discard = 1U;
  uart_rx_arm();
}
