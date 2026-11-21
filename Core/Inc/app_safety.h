/**
  ******************************************************************************
  * @file    app_safety.h
  * @brief   Achizitie senzori + protectii critice + protocol UART cu ESP32
  *          Placa: STM32F429I-DISC1 (STM32F429ZIT6), STM32 HAL
  *
  *  ADC1 (scan, 4 canale, 12 biti, DMA circular -> adc_dma_buf[4]):
  *    Rank 1  PA1  ADC1_IN1   Tensiune retea (divizor 10k / 2.2k)
  *    Rank 2  PA2  ADC1_IN2   Curent ACS712-05B (divizor 10k / 20k)
  *    Rank 3  PA3  ADC1_IN3   Fum/gaz MQ-2 (divizor 10k / 20k)
  *    Rank 4  PC3  ADC1_IN13  Temperatura LM35
  *
  *  GPIO:  PG2 = RELAY_MAIN, PG3 = RELAY_LIGHT (avarie), PD11 = BUZZER
  *  UART6: PC6 = TX, PC7 = RX, 115200 8N1
  *
  *  Utilizare in main.c:
  *    USER CODE BEGIN Includes : #include "app_safety.h"
  *    USER CODE BEGIN 2        : App_Safety_Init();
  *    USER CODE BEGIN 3        : App_Safety_Process();
  ******************************************************************************
  */
#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ============================ Configurare iesiri ============================ */
/* 1 = releul se anclanseaza cu nivel LOW pe pin (module releu uzuale cu optocuplor) */
#ifndef RELAY_ACTIVE_LOW
#define RELAY_ACTIVE_LOW            1
#endif
/* Releul 2 (lumina de avarie, PG3) - implicit aceeasi polaritate ca releul 1 */
#ifndef LIGHT_ACTIVE_LOW
#define LIGHT_ACTIVE_LOW            RELAY_ACTIVE_LOW
#endif
/* 1 = buzzer activ cu nivel HIGH pe PD11 */
#ifndef BUZZER_ACTIVE_HIGH
#define BUZZER_ACTIVE_HIGH          1
#endif

/* ============================ Configurare ADC ============================== */
#define ADC_CH_COUNT                4U
#define ADC_IDX_VOLTAGE             0U      /* Rank 1 - PA1 */
#define ADC_IDX_CURRENT             1U      /* Rank 2 - PA2 */
#define ADC_IDX_SMOKE               2U      /* Rank 3 - PA3 */
#define ADC_IDX_TEMP                3U      /* Rank 4 - PC3 */

#define ADC_VREF_V                  3.3f
#define ADC_FULL_SCALE              4095.0f

#define FILTER_LEN                  64U     /* medie mobila (putere a lui 2) */
#define CURRENT_FAST_LEN            4U      /* fereastra scurta pentru scurtcircuit */
#define SAMPLE_PERIOD_MS            1U      /* 1 esantion / canal / ms */

/* Tensiune: divizor 10k (spre +12V) / 2.2k (spre GND) -> (10 + 2.2) / 2.2 */
#define VOLT_DIV_FACTOR             5.5454f

/* ACS712-05B: 185 mV/A, alimentat la 5V, apoi divizor 2/3 */
#define ACS_SENS_V_PER_A            0.12333f
#define ACS_ZERO_NOMINAL_V          1.667f
#define ACS_ZERO_MIN_V              1.40f   /* plaja acceptata la auto-calibrare */
#define ACS_ZERO_MAX_V              1.95f
#define CAL_SETTLE_MS               50U     /* asteptare dupa pornirea ADC */
#define CAL_SAMPLES                 200U    /* esantioane pentru offset-ul de zero */
#define CURRENT_DISPLAY_DEADBAND_A  0.03f   /* doar pentru afisare (zgomot) */

/* MQ-2: 0.0 - 3.0 V pe pin -> 0 - 100 % */
#define MQ2_FULL_SCALE_V            3.0f
/* Perioada de incalzire MQ-2 dupa pornire, cand fumul NU declanseaza alarma
   (temperatura ramane activa). Pune 0 pentru a dezactiva. */
#define MQ2_WARMUP_MS               30000U

/* LM35: 10 mV / grad C */
#define LM35_DEG_PER_V              100.0f

/* ============================ Praguri protectii ============================ */
#define I_SHORT_MAX                 3.50f   /* A - scurtcircuit, instantaneu */
#define I_OVERLOAD_DEFAULT          1.50f   /* A - suprasarcina (modificabil prin UART) */
#define I_OVERLOAD_MIN              0.10f   /* A - limita inferioara acceptata la SET_IMAX */
#define OVERLOAD_TRIP_MS            2000U   /* ms consecutive peste prag */

#define SMOKE_ALARM_PCT             50.0f   /* % */
#define TEMP_ALARM_C                30.0f   /* grade C (prag de demonstratie) */
#define FIRE_CONFIRM_MS             500U    /* conditia trebuie sa persiste (anti-zgomot) */
#define FIRE_BLINK_MS               300U    /* perioada de comutare PG3 */

/* ============================ Protocol UART ================================ */
#define TELEMETRY_PERIOD_MS         500U
#define UART_RX_LINE_MAX            64U
#define UART_TX_BUF_LEN             96U

/* ============================ Tipuri publice =============================== */
typedef enum
{
  FAULT_NONE          = 0,
  FAULT_OVERLOAD      = 1,
  FAULT_SHORT_CIRCUIT = 2,
  FAULT_FIRE          = 3
} FaultCode_t;

typedef struct
{
  float       voltage_v;
  float       current_a;
  float       temp_c;
  float       smoke_pct;
  FaultCode_t fault;
  uint8_t     relay_on;
  uint8_t     calibrated;      /* 1 dupa terminarea auto-calibrarii ACS712 */
  uint8_t     calib_ok;        /* 0 = offset in afara plajei, s-a folosit valoarea nominala */
  float       acs_zero_v;
  float       i_overload_max;
} SafetyStatus_t;

/* Buffer DMA (scris continuu de DMA2 Stream0) */
extern volatile uint16_t adc_dma_buf[ADC_CH_COUNT];

/* ============================ API =========================================== */
void App_Safety_Init(void);                      /* o data, dupa MX_xxx_Init() */
void App_Safety_Process(void);                   /* in while(1), non-blocant   */
void App_Safety_GetStatus(SafetyStatus_t *out);  /* instantaneu (debug / LCD)  */

#ifdef __cplusplus
}
#endif

#endif /* APP_SAFETY_H */
