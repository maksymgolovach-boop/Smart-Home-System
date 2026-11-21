# Sistem_SmartHome – setări STM32CubeMX și integrare

Placă: **STM32F429I-DISC1** (STM32F429ZIT6, LQFP144). Proiect CubeMX 6.19, toolchain **CMake**.
Legendă: ✅ = deja setat corect în `Sistem_SmartHome.ioc` · ⚠️ = de modificat.

## 1. System Core

| Periferic | Setare | Valoare | Stare |
|---|---|---|---|
| SYS | Debug | Serial Wire (PA13/PA14) | ✅ |
| SYS | Timebase Source | SysTick | ✅ |
| RCC | Sursa ceasului (Clock Configuration) | HSI → PLL (M=8, N=180, P=2) → SYSCLK 180 MHz, Over-Drive activ | ✅ |
| RCC | APB2 | /2 → 90 MHz (ADC clock = 90/4 = 22.5 MHz, max 36 MHz) | ✅ |

## 2. ADC1 (Analog → ADC1)

Canale bifate: **IN1 (PA1), IN2 (PA2), IN3 (PA3), IN13 (PC3)**

| Parametru | Valoare | Stare |
|---|---|---|
| Clock Prescaler | PCLK2 divided by 4 | ✅ |
| Resolution | 12 bits | ✅ |
| Data Alignment | Right | ✅ |
| Scan Conversion Mode | Enabled | ✅ |
| Continuous Conversion Mode | Enabled | ✅ |
| Discontinuous Conversion Mode | Disabled | ✅ |
| DMA Continuous Requests | **Enabled** | ✅ |
| End Of Conversion Selection | EOC flag at the end of single channel conversion | ✅ |
| Number Of Conversion | 4 | ✅ |
| External Trigger Conversion Source | Regular Conversion launched by software | ✅ |
| Rank 1 | Channel 1, 84 Cycles | ✅ |
| Rank 2 | Channel 2, 84 Cycles | ✅ |
| Rank 3 | Channel 3, 84 Cycles | ✅ |
| Rank 4 | Channel 13, 84 Cycles | ✅ |

**DMA Settings (tab-ul ADC1):** ADC1 → DMA2 Stream 0, Peripheral To Memory, **Mode = Circular**,
Increment Address: Memory ✔ / Peripheral ✘, Data Width: **Half Word / Half Word**, FIFO dezactivat. ✅

## 3. USART6 (Connectivity → USART6)

| Parametru | Valoare | Stare |
|---|---|---|
| Mode | Asynchronous | ✅ |
| Hardware Flow Control | Disable | ✅ |
| Baud Rate | 115200 | ✅ |
| Word Length | 8 Bits | ✅ |
| Parity | None | ✅ |
| Stop Bits | 1 | ✅ |
| Data Direction | Receive and Transmit | ✅ |
| Over Sampling | 16 Samples | ✅ |
| Pini | PC6 = USART6_TX (→ RX ESP32), PC7 = USART6_RX (← TX ESP32) | ✅ |
| GPIO Settings → PC7 | Pull-up (recomandat: linia RX nu rămâne în aer cât ESP32 e în reset) | ⚠️ opțional |

## 4. NVIC

| Întrerupere | Enabled | Stare |
|---|---|---|
| DMA2 stream0 global interrupt | ✔ (forțat de CubeMX; codul dezactivează HT/TC la runtime) | ✅ |
| USART6 global interrupt | ✔ – **obligatoriu** pentru `HAL_UART_Receive_IT` / `Transmit_IT` | ✅ |

## 5. GPIO (System Core → GPIO)

| Pin | User Label | Mode | Pull | Speed | GPIO output level | Stare |
|---|---|---|---|---|---|---|
| PD11 | `BUZZER` | Output Push Pull | No pull | Low | Low | ✅ |
| PG2 | `RELAY_MAIN` | Output Push Pull | No pull | Low | **High** (cu `RELAY_ACTIVE_LOW 1`) | ⚠️ acum e Low |
| PG3 | `RELAY_LIGHT` | Output Push Pull | No pull | Low | **High** (cu `RELAY_ACTIVE_LOW 1`) | ⚠️ acum e Low |

> Cu modul de relee activ pe LOW, nivelul inițial Low anclanșează releele pentru câteva microsecunde,
> între `MX_GPIO_Init()` și `App_Safety_Init()`. Setează **High** ca releele să rămână decuplate de la reset.
> Etichetele (User Label) trebuie păstrate exact, pentru că `app_safety.c` folosește `RELAY_MAIN_Pin`, `RELAY_LIGHT_Pin` și `BUZZER_Pin` din `main.h`.

## 6. Project Manager

| Setare | Valoare | Stare |
|---|---|---|
| Toolchain / IDE | CMake | ✅ |
| Code Generator → Generate peripheral initialization as a pair of '.c/.h' files | ✔ | ✅ |
| Linker Settings → Minimum Heap Size | **0x400** (pentru `snprintf` cu `%f`) | ⚠️ acum 0x200 |
| Linker Settings → Minimum Stack Size | **0x800** | ⚠️ acum 0x400 |

## 7. Suport float în `snprintf` (`-u _printf_float`)

- **Proiect CMake (cazul tău):** în `CMakeLists.txt` (fișierul din rădăcină, pe care CubeMX nu-l regenerează):
  ```cmake
  target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
  ```
  ✅ Linia există deja. `Core/Src/app_safety.c` a fost adăugat în `target_sources(... # Add user sources here)`.
- **Proiect STM32CubeIDE clasic (managed build):** Project → Properties → C/C++ Build → Settings →
  MCU/MPU Settings → bifează **"Use float with printf from newlib-nano (-u _printf_float)"**.

## 8. Integrare în `main.c` (doar în blocurile USER CODE)

```c
/* USER CODE BEGIN Includes */
#include "app_safety.h"
/* USER CODE END Includes */

  /* USER CODE BEGIN 2 */
  App_Safety_Init();   /* calibrare ACS712, pornire ADC+DMA, receptie UART */
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Safety_Process(); /* non-blocant: senzori, protectii, protocol UART */
  }
  /* USER CODE END 3 */
```

`HAL_UART_RxCpltCallback` și `HAL_UART_ErrorCallback` sunt definite în `app_safety.c`. Nu le defini și în alt fișier.

## 9. ⚠️ Conflicte de pini pe STM32F429I-DISC1 (UM1670)

| Pin | Conectat pe placă la | Efect |
|---|---|---|
| **PA1** (Tensiune) | giroscop INT1 (L3GD20 / I3G4250D) | Ieșirea giroscopului forțează pinul → citire ADC eronată |
| **PA2** (Curent) | giroscop INT2 | la fel – afectează direct protecțiile de curent |
| **PD11** (Buzzer) | LCD TE (ieșire a controlerului LCD) | două ieșiri pe același fir |
| PA3 (Fum) | LCD B5 (intrare LCD) | de regulă acceptabil |
| PC6 / PC7 (UART6) | LCD HSYNC / G6 (intrări LCD) | funcționează; LCD-ul poate afișa artefacte |

Pini fără funcție pe placă în UM1670 (de verificat în tabelul din manual înainte de mutare): PA5 (ADC1_IN5), PC3, PG2, PG3, PF6.
Dacă muți un canal ADC, păstrează **ordinea rank-urilor** (1 = tensiune, 2 = curent, 3 = fum, 4 = temperatură). Codul nu depinde de pin, doar de rank.

## 10. Comportament implementat (rezumat)

- Pornire: toate ieșirile decuplate → 50 ms stabilizare → 200 eșantioane (1/ms) pentru offset-ul ACS712 (acceptat între 1.40–1.95 V, altfel se folosește 1.667 V) → **PG2 se anclanșează automat**.
- Filtrare: medie mobilă pe 64 de eșantioane (1 kHz) pe fiecare canal. Scurtcircuitul se evaluează pe o fereastră de 4 ms (reacție măsurată în simulare: 4 ms).
- Avariile rămân memorate până la `CMD,RESET_ALARM`. Codul raportat respectă prioritatea **2 (scurtcircuit) > 3 (incendiu) > 1 (suprasarcină)**.
- Incendiul nu decuplează PG2 (conform specificației), ci pornește buzzerul și lumina PG3 intermitentă la 300 ms.
- Parametri suplimentari în `app_safety.h`, ușor de schimbat:
  - `FIRE_CONFIRM_MS = 500`: anti-zgomot pentru alarma de incendiu;
  - `MQ2_WARMUP_MS = 30000`: fumul e ignorat 30 s după pornire, cât se încălzește MQ-2; temperatura rămâne activă. Pune 0 ca să dezactivezi.
- `CMD,SET_IMAX,<v>` este acceptat doar pentru 0.10 ≤ v < 3.50 A. `CMD,RELAY,x` este acceptat doar fără avarie și după calibrare. După orice comandă se trimite imediat un cadru `DATA,...`.
