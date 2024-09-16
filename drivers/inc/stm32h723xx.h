/*
 * STM32H743ZG_peripherals.h
 *
 *  Created on: Sep 29, 2023
 *      Author: jbates
 */

#ifndef STM32H723ZG_PERIPHERALS_H_
#define STM32H723ZG_PERIPHERALS_H_

#include <stdint.h>

#define __vo volatile // For easy casting as volatile memory

/************** START: Processor specific details (cortex-m7) *****************/
/**
 * ARM Cortex M7 processor NVIC ISERx and ICERx register addresses
 */
#define NVIC_ISERX_BASEADDR 0xE000E100
#define NVIC_ICERX_BASEADDR 0xE000E180

typedef struct NVIC_RegDef {
  __vo uint32_t ISER[8]; // NVIC Interrupt set registers
  uint32_t RESERVED1[24];
  __vo uint32_t ICER[8]; // NVIC Interrupt clear registers
  uint32_t RESERVED2[24];
  __vo uint32_t ISPR[8]; // NVIC Interrupt set pending registers
  uint32_t RESERVED3[24];
  __vo uint32_t ICPR[8]; // NVIC Interrupt clear pending registers
  uint32_t RESERVED4[24];
  __vo uint32_t IABR[8]; // NVIC Interrupt active bit register (read only)
  uint32_t RESERVED5[56];
  __vo uint32_t IPR[60]; // NVIC Interrupt priority register
} NVIC_RegDef_t;

#define NVIC ((NVIC_RegDef_t *)NVIC_ISERX_BASEADDR)

/************** END: Processor specific details (cortex-m7) *****************/

// Base addresses of peripherals
#define HSEM_BASEADDR 0x58026400
#define ADC3_BASEADDR 0x58026000
#define DMAMUX2_BASEADDR 0x58025800
#define BDMA_BASEADDR 0x58025400
#define CRC_BASEADDR 0x58024C00
#define PWR_BASEADDR 0x58024800
#define RCC_BASEADDR 0x58024400

#define GPIOK_BASEADDR 0x58022800
#define GPIOJ_BASEADDR 0x58022400
#define GPIOH_BASEADDR 0x58021C00
#define GPIOG_BASEADDR 0x58021800
#define GPIOF_BASEADDR 0x58021400
#define GPIOE_BASEADDR 0x58021000
#define GPIOD_BASEADDR 0x58020C00
#define GPIOC_BASEADDR 0x58020800
#define GPIOB_BASEADDR 0x58020400
#define GPIOA_BASEADDR 0x58020000

#define DTS_BASEADDR 0x58006800
#define SAI4_BASEADDR 0x58005400
#define IWDG_BASEADDR 0x58004800
#define RTC_BASEADDR 0x58004000
#define VREF_BASEADDR 0x58003C00
#define COMP1_BASEADDR 0x58003800
#define LPTIM5_BASEADDR 0x58003000
#define LPTIM4_BASEADDR 0x58002C00
#define LPTIM3_BASEADDR 0x58002800
#define LPTIM2_BASEADDR 0x58002400
#define I2C4_BASEADDR 0x58001C00
#define SPI_I2S_6_BASEADDR 0x58001400
#define LPUART1_BASEADDR 0x58000C00
#define SYSCFG_BASEADDR 0x58000400
#define EXTI_BASEADDR 0x58000000

#define OTFDEC2_BASEADDR 0x5200BC00
#define OTFDEC1_BASEADDR 0x5200B800
#define OCTOSPI_IO_BASEADDR 0x5200B400
#define OCTOSPI2_DELAY_BASEADDR 0x5200B000
#define OCTOSPI2_BASEADDR 0x5200A000
#define RAMECC1_BASEADDR 0x52009000
#define SDMMC1_DELAY_BASEADDR 0x52008000
#define SDMMC1_BASEADDR 0x52007000
#define OCTOSPI1_DELAY_BASEADDR 0x52006000
#define OCTOSPI1_CTRL_BASEADDR 0x52005000
#define FMC_CTRL_BASEADDR 0x52004000
#define FLASH_INTERFACE_BASEADDR 0x52002000
#define DMA2D_CHROM_ART_BASEADDR 0x52001000
#define MDMA_BASEADDR 0x52000000
#define GPV_BASEADDR 0x51000000
#define WWDG_BASEADDR 0x50003000
#define LTDC_BASEADDR 0x50001000
#define CORDIC_BASEADDR 0x48024400
#define FMAC_BASEADDR 0x48024000
#define RAMECC_BASEADDR 0x48023000
#define SDMMC2_DELAY_BASEADDR 0x48022800
#define SDMMC2_BASEADDR 0x48022400
#define RNG_BASEADDR 0x48021800
#define HASH_BASEADDR 0x48021400
#define CRYPTO_BASEADDR 0x48021000
#define PSSI_BASEADDR 0x48020400
#define DCMI_BASEADDR 0x48020000

#define USB1_OTG_HS_BASEADDR 0x40040000
#define ETHERNET_MAC_BASEADDR 0x40028000
#define ADC1_ADC2_BASEADDR 0x40022000
#define DMAMUX1_BASEADDR 0x40020800
#define DMA2_BASEADDR 0x40020400
#define DMA1_BASEADDR 0x40020000
#define DFSDM1_BASEADDR 0x40017800
#define SAI1_BASEADDR 0x40015800
#define SPI5_BASEADDR 0x40015000
#define TIM17_BASEADDR 0x40014800
#define TIM16_BASEADDR 0x40014400
#define TIM15_BASEADDR 0x40014000
#define SPI4_BASEADDR 0x40013400
#define SPI1_I2S1_BASEADDR 0x40013000
#define USART10_BASEADDR 0x40011C00
#define UART9_BASEADDR 0x40011800
#define USART6_BASEADDR 0x40011400
#define USART1_BASEADDR 0x40011000
#define TIM8_BASEADDR 0x40010400
#define TIM1_BASEADDR 0x40010000

#define TIM24_BASEADDR 0x4000E400
#define TIM23_BASEADDR 0x4000E000
#define FDCAN3_BASEADDR 0x4000D400
#define CAN_BASEADDR 0x4000AC00
#define CAN_CCU_BASEADDR 0x4000A800
#define FDCAN2_BASEADDR 0x4000A400
#define FDCAN1_BASEADDR 0x4000A000
#define MDIOS_BASEADDR 0x40009400
#define OPAMP_BASEADDR 0x40009000
#define SWPMI_BASEADDR 0x40008800
#define CRS_BASEADDR 0x40008400
#define UART8_BASEADDR 0x40007C00
#define UART7_BASEADDR 0x40007800
#define DAC1_BASEADDR 0x40007400
#define HDMI_CEC_BASEADDR 0x40006C00
#define I2C5_BASEADDR 0x40006400
#define I2C3_BASEADDR 0x40005C00
#define I2C2_BASEADDR 0x40005800
#define I2C1_BASEADDR 0x40005400
#define UART5_BASEADDR 0x40005000
#define UART4_BASEADDR 0x40004C00
#define USART3_BASEADDR 0x40004800
#define USART2_BASEADDR 0x40004400
#define SPDIFRX1_BASEADDR 0x40004000
#define SPI3_I2S3_BASEADDR 0x40003C00
#define SPI2_I2S2_BASEADDR 0x40003800
#define LPTIM1_BASEADDR 0x40002400
#define TIM14_BASEADDR 0x40002000
#define TIM13_BASEADDR 0x40001C00
#define TIM12_BASEADDR 0x40001800

#define TIM7_BASEADDR 0x40001400
#define TIM6_BASEADDR 0x40001000
#define TIM5_BASEADDR 0x40000C00
#define TIM4_BASEADDR 0x40000800
#define TIM3_BASEADDR 0x40000400
#define TIM2_BASEADDR 0x40000000

// IRQ NVIC Position numbers
typedef enum {
  IRQ_NO_WWDG1 = 0,
  IRQ_NO_PVD_PVM,
  IRQ_NO_RTC_TAMP_STAMP_CSS__LSE,
  IRQ_NO_RTC_WKUP,
  IRQ_NO_FLASH,
  IRQ_NO_RCC,
  IRQ_NO_EXTIO,
  IRQ_NO_EXTI1,
  IRQ_NO_EXT12,
  IRQ_NO_EXT13,
  IRQ_NO_EXT14,
  IRQ_NO_DMA1_STRO,
  IRQ_NO_DMA1_STR1,
  IRQ_NO_DMA1_STR2,
  IRQ_NO_DMA1_STR3,
  IRQ_NO_DMA1_STR4,
  IRQ_NO_DMA1_STR5,
  IRQ_NO_DMA1_STR6,
  IRQ_NO_ADC1_2,
  IRQ_NO_FDCAN1_ITO,
  IRQ_NO_FDCAN2_ITO,
  IRQ_NO_FDCAN1_IT1,
  IRQ_NO_FDCAN2_IT1,
  IRQ_NO_EXTI9_5,
  IRQ_NO_TIM1_BRK,
  IRQ_NO_TIM1_UP,
  IRQ_NO_TIM1_TRG_COM,
  IRQ_NO_TIM1_CC,
  IRQ_NO_TIM2,
  IRQ_NO_TIM3,
  IRQ_NO_TIM4,
  IRQ_NO_I2C1_EV,
  IRQ_NO_I2C1_ER,
  IRQ_NO_I2C2_EV,
  IRQ_NO_I2C2_ER,
  IRQ_NO_SPI1,
  IRQ_NO_SPI2,
  IRQ_NO_USART1,
  IRQ_NO_USART2,
  IRQ_NO_USART3,
  IRQ_NO_EXTI15_10,
  IRQ_NO_RTC_ALARM,
  IRQ_NO_RESERVED1,
  IRQ_NO_TIM8_BRK__TIM12,
  IRQ_NO_TIM8_UP_TIM13,
  IRQ_NO_TIM8_TRG__COM_TIM14,
  IRQ_NO_TIM8_CC,
  IRQ_NO_DMA1_STR7,
  IRQ_NO_FMC,
  IRQ_NO_SDMMC1,
  IRQ_NO_TIM5,
  IRQ_NO_SPI3,
  IRQ_NO_UART4,
  IRQ_NO_UART5,
  IRQ_NO_TIM6_DAC,
  IRQ_NO_TIM7,
  IRQ_NO_DMA2_STRO,
  IRQ_NO_DMA2_STR1,
  IRQ_NO_DMA2_STR2,
  IRQ_NO_DMA2_STR3,
  IRQ_NO_DMA2_STR4,
  IRQ_NO_ETH,
  IRQ_NO_ETH_WKUP,
  IRQ_NO_FDCAN_CAL,
  IRQ_NO_RESERVED2,
  IRQ_NO_RESERVED3,
  IRQ_NO_RESERVED4,
  IRQ_NO_RESERVED5,
  IRQ_NO_DMA2_STR5,
  IRQ_NO_DMA2_STR6,
  IRQ_NO_DMA2_STR7,
  IRQ_NO_USART6,
  IRQ_NO_I2C3_EV,
  IRQ_NO_I2C3_ER,
  IRQ_NO_OTG_HS_EP1__OUT,
  IRQ_NO_OTG_HS_EP1__IN,
  IRQ_NO_OTG_HS_WKUP,
  IRQ_NO_OTG_HS,
  IRQ_NO_DCMI_PSSI,
  IRQ_NO_CRYP,
  IRQ_NO_HASH_RNG,
  IRQ_NO_FPU,
  IRQ_NO_UART7,
  IRQ_NO_UART8,
  IRQ_NO_SP14,
  IRQ_NO_SPI5,
  IRQ_NO_SPI6,
  IRQ_NO_SAI1,
  IRQ_NO_LTDC,
  IRQ_NO_LTDC_ERR,
  IRQ_NO_DMA2D,
  IRQ_NO_RESERVED6,
  IRQ_NO_OCTOSPI1,
  IRQ_NO_LPTIM1,
  IRQ_NO_CEC,
  IRQ_NO_I2C4_EV,
  IRQ_NO_I2C4_ER,
  IRQ_NO_SPDIF,
  IRQ_NO_RESERVED7,
  IRQ_NO_RESERVED8,
  IRQ_NO_RESERVED9,
  IRQ_NO_RESERVED10,
  IRQ_NO_DMAMUX1_OV,
  IRQ_NO_RESERVED11,
  IRQ_NO_RESERVED12,
  IRQ_NO_RESERVED13,
  IRQ_NO_RESERVED14,
  IRQ_NO_RESERVED15,
  IRQ_NO_RESERVED16,
  IRQ_NO_RESERVED17,
  IRQ_NO_DFSDM1_FLTO,
  IRQ_NO_DFSDM1_FLT1,
  IRQ_NO_DFSDM1_FLT2,
  IRQ_NO_DFSDM1_FLT3,
  IRQ_NO_RESERVED18,
  IRQ_NO_SWPMI1,
  IRQ_NO_TIM15,
  IRQ_NO_TIM16,
  IRQ_NO_TIM17,
  IRQ_NO_MDIOS_WKUP,
  IRQ_NO_MDIOS,
  IRQ_NO_RESERVED19,
  IRQ_NO_MDMA,
  IRQ_NO_RESERVED20,
  IRQ_NO_SDMMC2,
  IRQ_NO_HSEMO,
  IRQ_NO_RESERVED21,
  IRQ_NO_ADC3,
  IRQ_NO_DMAMUX2_OVR,
  IRQ_NO_BDMA_CHO,
  IRQ_NO_BDMA_CH1,
  IRQ_NO_BDMA_CH2,
  IRQ_NO_BDMA_CH3,
  IRQ_NO_BDMA_CH4,
  IRQ_NO_BDMA_CH5,
  IRQ_NO_BDMA_CH6,
  IRQ_NO_BDMA_CH7,
  IRQ_NO_COMP,
  IRQ_NO_LPTIM2,
  IRQ_NO_LPTIM3,
  IRQ_NO_LPTIM4,
  IRQ_NO_LPTIM5,
  IRQ_NO_LPUART,
  IRQ_NO_RESERVED22,
  IRQ_NO_CRS,
  IRQ_NO_ECC_DIAG_IT,
  IRQ_NO_SAI4,
  IRQ_NO_TEMP_IT,
  IRQ_NO_RESERVED23,
  IRQ_NO_WKUP,
  IRQ_NO_OCTOSPI2,
  IRQ_NO_OTFDEC1,
  IRQ_NO_OTFDEC2,
  IRQ_NO_FMAC,
  IRQ_NO_CORDIC_IT,
  IRQ_NO_UART9,
  IRQ_NO_USART10,
  IRQ_NO_I2C5_EV,
  IRQ_NO_I2C5_ER,
  IRQ_NO_FDCAN3_ITO,
  IRQ_NO_FDCAN3_IT1,
  IRQ_NO_TIM23,
  IRQ_NO_TIM24
} NVIC_INTERRUPT_POSITIONS;

// -------------peripheral structs-------------
// Use these with and declare them as pointer structs to
// be able to access the memory easily

typedef struct RCC_RegDef {
  __vo uint32_t CR; // 0
  __vo uint32_t HSICFGR;
  __vo uint32_t CRRCR;
  __vo uint32_t CSICFGR;
  __vo uint32_t CFGR; // 10
  uint32_t RESERVED1;
  __vo uint32_t D1CFGR; // good
  __vo uint32_t D2CFGR;
  __vo uint32_t D3CFGR; // 20
  uint32_t RESERVED2;
  __vo uint32_t PLLCKSELR;
  __vo uint32_t PLLCFGR;
  __vo uint32_t PLL1DIVR; // 30
  __vo uint32_t PLL1FRACR;
  __vo uint32_t PLL2DIVR;
  __vo uint32_t PLL2FRACR;
  __vo uint32_t PLL3DIVR; // 40
  __vo uint32_t PLL3FRACR;
  uint32_t RESERVED3;
  __vo uint32_t D1CCIPR;  // good
  __vo uint32_t D2CCIP1R; // 50
  __vo uint32_t D2CCIP2R;
  __vo uint32_t D3CCIPR;
  uint32_t RESERVED4;
  __vo uint32_t CIER; // 60
  __vo uint32_t CIFR;
  __vo uint32_t CICR;
  uint32_t RESERVED5;
  __vo uint32_t BDCR; // good //70
  __vo uint32_t CSR;
  uint32_t RESERVED6;
  __vo uint32_t AHB3RSTR;
  __vo uint32_t AHB1RSTR; // 80
  __vo uint32_t AHB2RSTR;
  __vo uint32_t AHB4RSTR;
  __vo uint32_t APB3RSTR;
  __vo uint32_t APB1LRSTR; // 90
  __vo uint32_t APB1HRSTR; // good
  __vo uint32_t APB2RSTR;
  __vo uint32_t APB4RSTR;
  __vo uint32_t GCR; // A0
  uint32_t RESERVED7;
  __vo uint32_t D3AMR;
  uint32_t RESERVED8[9];
  __vo uint32_t RSR; // D0
  __vo uint32_t AHB3ENR;
  __vo uint32_t AHB1ENR; // good
  __vo uint32_t AHB2ENR;
  __vo uint32_t AHB4ENR; // E0
  __vo uint32_t APB3ENR;
  __vo uint32_t APB1LENR;
  __vo uint32_t APB1HENR;
  __vo uint32_t APB2ENR; // F0
  __vo uint32_t APB4ENR;
  uint32_t RESERVED9;
  __vo uint32_t AHB3LPENR;
  __vo uint32_t AHB1LPENR; // good 00
  __vo uint32_t AHB2LPENR;
  __vo uint32_t AHB4LPENR;
  __vo uint32_t APB3LPENR;
  __vo uint32_t APB1LLPENR; // 10
  __vo uint32_t APB1HLPENR;
  __vo uint32_t APB2LPENR; // good
  __vo uint32_t APB4LPENR;
  uint32_t RESERVED10[4];
  __vo uint32_t C1_RSR; // 30
  __vo uint32_t C1_AHB3ENR;
  __vo uint32_t C1_AHB1ENR;
  __vo uint32_t C1_AHB2ENR;
  __vo uint32_t C1_AHB4ENR; // 40
  __vo uint32_t C1_APB3ENR; // good
  __vo uint32_t C1_APB1LENR;
  __vo uint32_t C1_APB1HENR;
  __vo uint32_t C1_APB2ENR; // 50
  __vo uint32_t C1_APB4ENR;
  uint32_t RESERVED11;
  __vo uint32_t C1_AHB3LPENR;
  __vo uint32_t C1_AHB1LPENR; // 60
  __vo uint32_t C1_AHB2LPENR;
  __vo uint32_t C1_AHB4LPENR;
  __vo uint32_t C1_APB3LPENR;
  __vo uint32_t C1_APB1LLPENR; // 70
  __vo uint32_t C1_APB1HLPENR;
  __vo uint32_t C1_APB2LPENR;
  __vo uint32_t C1_APB4LPENR;
} RCC_RegDef_t;

typedef struct HSEM_RegDef {
  __vo uint32_t HSEM_R[32];   // HSEM register semaphore
  __vo uint32_t HSEM_RLR[32]; // HSEM read lock register semaphore
  __vo uint32_t HSEM_IER;     // HSEM interrupt enable register
  __vo uint32_t HSEM_ICR;     // HSEM interrupt clear register
  __vo uint32_t HSEM_ISR;     // HSEM interrupt status register
  __vo uint32_t HSEM_MISR;    // HSEM master interrupt status register
  __vo uint32_t HSEM_CR;      // HSEM clear register
  __vo uint32_t HSEM_KEYR;    // HSEM interrupt clear register
} HSEM_RegDef_t;

typedef struct GPIO_RegDef {
  __vo uint32_t MODER;   // GPIO Mode register
  __vo uint32_t OTYPER;  // GPIO Output type register
  __vo uint32_t OSPEEDR; // GPIO Output speed register
  __vo uint32_t PUPDR;   // GPIO Pull-up/Pull-down register
  __vo uint32_t IDR;     // GPIO Input data register
  __vo uint32_t ODR;     // GPIO Output data register
  __vo uint32_t BSRR;    // GPIO Bit set/reset register
  __vo uint32_t LCKR;    // GPIO Port configuration lock register
  __vo uint32_t AFR[2];  // GPIO Alternate function registers
} GPIO_RegDef_t;

typedef struct SYSCFG_RegDef {
  uint32_t RESERVED1;
  __vo uint32_t PMCR; // SysConfig peripheral mode configuration register
  __vo uint32_t
      EXTICR[4];         // SysConfig external interrupt configuration registers
  uint32_t RESERVED2[2]; // Reserved
  __vo uint32_t CCCSR;   // SysConfig compensation cell control/status register
  __vo uint32_t CCVR;    // SysConfig compensation cell value register
  __vo uint32_t CCCR;    // SysConfig compensation cell code register
  uint32_t RESERVED3;    // Reserved
  __vo uint32_t
      ADC2ALT; // SysConfig internal input alternate connection register
  uint32_t RESERVED4[60];  // Reserved
  __vo uint32_t PKGR;      // SysConfig package register
  uint32_t RESERVED5[118]; // Reserved
  __vo uint32_t UR[18];    // SysConfig user registers
} SYSCFG_RegDef_t;

typedef struct EXTI_RegDef {
  __vo uint32_t RTSR1;       // EXTI Rising trigger selection register
  __vo uint32_t FTSR1;       // EXTI Falling trigger selection register
  __vo uint32_t SWIER1;      // EXTI Software interrupt event register
  __vo uint32_t D3PMR1;      // EXTI D3 Pending mask register
  __vo uint32_t D3PCR1L;     // EXTI D3 Pending clear selection register low
  __vo uint32_t D3PCR1H;     // EXTI D3 Pending clear selection register high
  uint32_t RESERVED1[2];     // Reserved
  __vo uint32_t RTSR2;       // EXTI Rising trigger selection register
  __vo uint32_t FTSR2;       // EXTI Falling trigger selection register
  __vo uint32_t SWIER2;      // EXTI Software interrupt event register
  __vo uint32_t D3PMR2;      // EXTI D3 Pending mask register
  __vo uint32_t D3PCR2L;     // EXTI D3 Pending clear selection register low
  __vo uint32_t D3PCR2H;     // EXTI D3 Pending clear selection register high
  uint32_t RESERVED2[2];     // Reserved
  __vo uint32_t RTSR3;       // EXTI Rising trigger selection register
  __vo uint32_t FTSR3;       // EXTI Falling trigger selection register
  __vo uint32_t SWIER3;      // EXTI Software interrupt event register
  __vo uint32_t D3PMR3;      // EXTI D3 Pending mask register
  __vo uint32_t D3PCR3L;     // EXTI D3 Pending clear selection register low
  __vo uint32_t D3PCR3H;     // EXTI D3 Pending clear selection register high
  uint32_t RESERVED3[10];    // Reserved
  __vo uint32_t CPUIMR1;     // EXTI Interrupt mask register
  __vo uint32_t CPUCPUE_MR1; // EXTI Interrupt clear register
  __vo uint32_t CPUPR1;      // EXTI Interrupt pending register
  uint32_t RESERVED4;        // Reserved
  __vo uint32_t CPUIMR2;     // EXTI Interrupt mask register
  __vo uint32_t CPUEMR2;     // EXTI Event mask register
  __vo uint32_t CPUPR2;      // EXTI Pending register
  uint32_t RESERVED5;        // Reserved
  __vo uint32_t CPUIMR3;     // EXTI Interrupt mask register
  __vo uint32_t CPUEMR3;     // EXTI Event mask register
  __vo uint32_t CPUPR3;      // EXTI Pending register
} EXTI_RegDef_t;

typedef struct SPI_RegDef {
  __vo uint32_t CR1;         // SPI control register 1
  __vo uint32_t CR2;         // SPI control register 2
  __vo uint32_t CFG1;        // SPI configuration register 1
  __vo uint32_t CFG2;        // SPI configuration register 2
  __vo uint32_t IER;         // SPI interrupt enable register
  __vo uint32_t SR;          // SPI status register
  __vo uint32_t IFCR;        // SPI interrupt and status register
  __vo uint32_t TXDR;        // SPI transmit data register
  uint32_t RESERVED0[3];     // Reserved
  __vo uint32_t RXDR;        // SPI receive data register
  uint32_t RESERVED1[3];     // Reserved
  __vo uint32_t CRCPOLY;     // SPI polynomial configuration register
  __vo uint32_t TXCRC;       // SPI transmit CRC register
  __vo uint32_t RXCRC;       // SPI receive CRC register
  __vo uint32_t UDRDR;       // SPI underrun data register
  __vo uint32_t SPI_I2SCFGR; // SPI_I2S configuration register

} SPI_RegDef_t;

typedef struct GenTimer_RegDef {
  __vo uint32_t CR1;   // Control Register 1
  __vo uint32_t CR2;   // Control Register 2
  __vo uint32_t SMCR;  // Slave Mode Control Register
  __vo uint32_t DIER;  // DMA/Interrupt Enable Register
  __vo uint32_t SR;    // Status Register
  __vo uint32_t EGR;   // Event Generation Register
  __vo uint32_t CCMR1; // Capture/Compare Mode Register 1
  __vo uint32_t CCMR2; // Capture/Compare Mode Register 2
  __vo uint32_t CCER;  // Capture/Compare Enable Register
  __vo uint32_t CNT;   // Counter
  __vo uint32_t PSC;   // Prescaler
  __vo uint32_t ARR;   // Auto-Reload Register
  __vo uint32_t RCR;   // Repetition Counter Register
  __vo uint32_t CCR1;  // Capture/Compare Register 1
  __vo uint32_t CCR2;  // Capture/Compare Register 2
  __vo uint32_t CCR3;  // Capture/Compare Register 3
  __vo uint32_t CCR4;  // Capture/Compare Register 4
  __vo uint32_t BDTR;  // Break and Dead-Time Register
  __vo uint32_t DCR;   // DMA Control Register
  __vo uint32_t DMAR;  // DMA Address for Full Transfer
  uint32_t RESERVED1;  // Reserved
  __vo uint32_t CCMR3; // Capture/Compare Mode Register 3
  __vo uint32_t CCR5;  // Capture/Compare Register 5
  __vo uint32_t CCR6;  // Capture/Compare Register 6
  __vo uint32_t AF1;   // Alternate Function Option Register 1
  __vo uint32_t AF2;   // Alternate Function Option Register 2
  __vo uint32_t TISEL; // Timer Input Selection Register
} GenTimer_RegDef_t;

//
// Actual Pointer Notation with base addresses:
#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)

#define HSEM_PCLK_EN() (RCC->AHB4ENR |= (1 << 25))
#define GPIOA_PCLK_EN() (RCC->AHB4ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB4ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB4ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB4ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB4ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB4ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB4ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB4ENR |= (1 << 7))
#define GPIOJ_PCLK_EN() (RCC->AHB4ENR |= (1 << 9))
#define GPIOK_PCLK_EN() (RCC->AHB4ENR |= (1 << 10))

#define SYSCFG_PCLK_EN() (RCC->APB4ENR |= (1 << 1))

#define HSEM ((HSEM_RegDef_t *)HSEM_BASEADDR)

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOJ ((GPIO_RegDef_t *)GPIOJ_BASEADDR)
#define GPIOK ((GPIO_RegDef_t *)GPIOK_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

// Generic macros
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

// SPI macros
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1LENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1LENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN() (RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN() (RCC->APB4ENR |= (1 << 5))

#define SPI1_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS() (RCC->APB1LENR &= ~(1 << 14))
#define SPI3_PCLK_DIS() (RCC->APB1LENR &= ~(1 << 15))
#define SPI4_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DIS() (RCC->APB4ENR &= ~(1 << 13))

#define SPI1_RESET()                                                           \
  do {                                                                         \
    (RCC->APB2RSTR |= (1 << 12));                                              \
    (RCC->APB2RSTR &= ~(1 << 12));                                             \
  } while (0)
#define SPI2_RESET()                                                           \
  do {                                                                         \
    (RCC->APB1LRSTR |= (1 << 14));                                             \
    (RCC->APB1LRSTR &= ~(1 << 14));                                            \
  } while (0)
#define SPI3_RESET()                                                           \
  do {                                                                         \
    (RCC->APB1LRSTR |= (1 << 15));                                             \
    (RCC->APB1LRSTR &= ~(1 << 15));                                            \
  } while (0)
#define SPI4_RESET()                                                           \
  do {                                                                         \
    (RCC->APB2RSTR |= (1 << 13));                                              \
    (RCC->APB2RSTR &= ~(1 << 13));                                             \
  } while (0)
#define SPI5_RESET()                                                           \
  do {                                                                         \
    (RCC->APB2RSTR |= (1 << 20));                                              \
    (RCC->APB2RSTR &= ~(1 << 20));                                             \
  } while (0)
#define SPI6_RESET()                                                           \
  do {                                                                         \
    (RCC->APB4RSTR |= (1 << 13));                                              \
    (RCC->APB4RSTR &= ~(1 << 13));                                             \
  } while (0)

#define SPI1 ((SPI_RegDef_t *)SPI1_I2S1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_I2S2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_I2S3_BASEADDR)
#define SPI4 ((SPI_RegDef_t *)SPI4_BASEADDR)
#define SPI5 ((SPI_RegDef_t *)SPI5_BASEADDR)
#define SPI6 ((SPI_RegDef_t *)SPI_I2S_6_BASEADDR)

/************** START: SPI register bit definitions *****************/
// SPI_CR1
typedef enum {
  SPI_CR1_SPE = 0,
  SPI_CR1_MASRSX = 8,
  SPI_CR1_CSTART,
  SPI_CR1_CSUSP,
  SPI_CR1_HDDIR,
  SPI_CR1_SSI,
  SPI_CR1_CRC33_17,
  SPI_CR1_RCRCINI,
  SPI_CR1_TCRCINI,
  SPI_CR1_IOLOCK
} SPI_CR1_BIT;

// SPI_CR2
typedef enum { SPI_CR2_TSIZE = 0, SPI_CR2_TSER = 16 } SPI_CR2_BIT;

// SPI_CFG1
typedef enum {
  SPI_CFG1_DSIZE = 0,
  SPI_CFG1_FTHLV = 5,
  SPI_CFG1_UDRCFG = 9,
  SPI_CFG1_UDRDET = 11,
  SPI_CFG1_RXDMAEN = 14,
  SPI_CFG1_TXDMAEN = 15,
  SPI_CFG1_CRCSIZE = 16,
  SPI_CFG1_CRCEN = 22,
  SPI_CFG1_MBR = 28
} SPI_CFG1_BIT;

// SPI_CFG2
typedef enum {
  SPI_CFG2_MSSI = 0,
  SPI_CFG2_MIDI = 4,
  SPI_CFG2_IOSWP = 15,
  SPI_CFG2_COMM = 17,
  SPI_CFG2_SP = 19,
  SPI_CFG2_MASTER = 22,
  SPI_CFG2_LSBFRST = 23,
  SPI_CFG2_CPHA = 24,
  SPI_CFG2_CPOL = 25,
  SPI_CFG2_SSM = 26,
  SPI_CFG2_SSIOP = 28,
  SPI_CFG2_SSOE = 29,
  SPI_CFG2_SSOM = 30,
  SPI_CFG2_AFCNTR = 31
} SPI_CFG2_BIT;

// SPI_IER
typedef enum {
  SPI_IER_RXPIE = 0,
  SPI_IER_TXPIE,
  SPI_IE_DXPIE,
  SPI_IER_EOTIE,
  SPI_IER_TXTFIE,
  SPI_IER_UDRIE,
  SPI_IER_OVRIE,
  SPI_IER_CRCEIE,
  SPI_IER_TIFREIE,
  SPI_IER_MODFIE,
  SPI_IER_TSERFIE
} SPI_IER_BIT;

// SPI_SR
typedef enum {
  SPI_SR_RXP = 0,
  SPI_SR_TXP,
  SPI_SR_DXP,
  SPI_SR_EOT,
  SPI_SR_TXTF,
  SPI_SR_UDR,
  SPI_SR_OVR,
  SPI_SR_CRCE,
  SPI_SR_TIFRE,
  SPI_SR_MODF,
  SPI_SR_TSERF,
  SPI_SR_SUSP,
  SPI_SR_TXC,
  SPI_SR_RXPLVL,
  SPI_SR_RXWNE = 15,
  SPI_SR_CTSIZE
} SPI_SR_BIT;

// SPI_IFCR
typedef enum {
  SPI_IFCR_EOTC = 3,
  SPI_IFCR_TXTFC,
  SPI_IFCR_UDRC,
  SPI_IFCR_OVRC,
  SPI_IFCR_CRCEC,
  SPI_IFCR_TIFREC,
  SPI_IFCR_MODFC,
  SPI_IFCR_TSERFC,
  SPI_IFCR_SUSPC
} SPI_IFCR_BIT;

// SPI_TXDR
typedef enum { SPI_TXDR_TXDR = 0 } SPI_TXDR_BIT;

// SPI_RXDR
typedef enum { SPI_RXDR_RXDR = 0 } SPI_RXDR_BIT;

// SPI_CRCPOLY
typedef enum { SPI_CRCPOLY_CRCPOLY = 0 } SPI_CRCPOLY_BIT;

// SPI_TXCRC
typedef enum { SPI_TXCRC_TXCRC = 0 } SPI_TXCRC_BIT;

// SPI_RXCRC
typedef enum { SPI_RXCRC_RXCRC = 0 } SPI_RXCRC_BIT;

// SPI_UDRDR
typedef enum { SPI_UDRDR_UDRDR = 0 } SPI_UDRDR_BIT;

// SPI_I2SCFGR
typedef enum {
  SPI_I2SCFGR_I2SMOD = 0,
  SPI_I2SCFGR_I2SCFG = 1,
  SPI_I2SCFGR_I2SSTD = 4,
  SPI_I2SCFGR_PCMSYNC = 7,
  SPI_I2SCFGR_DATLEN = 8,
  SPI_I2SCFGR_CHLEN = 10,
  SPI_I2SCFGR_CKPOL = 11,
  SPI_I2SCFGR_FIXCH = 12,
  SPI_I2SCFGR_WSINV = 13,
  SPI_I2SCFGR_DATFMT = 14,
  SPI_I2SCFGR_I2SDIV = 16,
  SPI_I2SCFGR_ODD = 24,
  SPI_I2SCFGR_MCKOE = 25
} SPI_I2SCFGR_BIT;

/************** END: SPI register bit definitions *****************/

/************** START: Timer register bit definitions ************/

// Timer macros
#define TIM1_PCLK_EN() (RCC->APB2ENR |= (1 << 0))
#define TIM2_PCLK_EN() (RCC->APB1LENR |= (1 << 0))
#define TIM3_PCLK_EN() (RCC->APB1LENR |= (1 << 1))
#define TIM4_PCLK_EN() (RCC->APB1LENR |= (1 << 2))
#define TIM5_PCLK_EN() (RCC->APB1LENR |= (1 << 3))
#define TIM6_PCLK_EN() (RCC->APB1LENR |= (1 << 4))
#define TIM7_PCLK_EN() (RCC->APB1LENR |= (1 << 5))
#define TIM8_PCLK_EN() (RCC->APB2ENR |= (1 << 1))
#define TIM12_PCLK_EN() (RCC->APB1LENR |= (1 << 6))
#define TIM13_PCLK_EN() (RCC->APB1LENR |= (1 << 7))
#define TIM14_PCLK_EN() (RCC->APB1LENR |= (1 << 8))
#define TIM16_PCLK_EN() (RCC->APB2ENR |= (1 << 17))
#define TIM15_PCLK_EN() (RCC->APB2ENR |= (1 << 16))
#define TIM17_PCLK_EN() (RCC->APB2ENR |= (1 << 18))
#define TIM23_PCLK_EN() (RCC->APB1HENR |= (1 << 24))
#define TIM24_PCLK_EN() (RCC->APB1HENR |= (1 << 25))

#define TIM1_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 0))
#define TIM2_PCLK_DIS() (RCC->APB1LENR &= ~(1 << 0))
#define TIM3_PCLK_DIS() (RCC->APB1LENR &= ~(1 << 1))
#define TIM4_PCLK_DIS() (RCC->APB1LENR &= ~(1 << 2))
#define TIM5_PCLK_DIS() (RCC->APB1LENR &= ~(1 << 3))
#define TIM6_PCLK_DIS() (RCC->APB1LENR &= ~(1 << 4))
#define TIM7_PCLK_DIS() (RCC->APB1LENR &= ~(1 << 5))
#define TIM8_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 1))
#define TIM12_PCLK_DIS() (RCC->APB1LENR &= ~(1 << 6))
#define TIM13_PCLK_DIS() (RCC->APB1LENR &= ~(1 << 7))
#define TIM14_PCLK_DIS() (RCC->APB1LENR &= ~(1 << 8))
#define TIM16_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 17))
#define TIM15_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 16))
#define TIM17_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 18))
#define TIM23_PCLK_DIS() (RCC->APB1HENR &= ~(1 << 24))
#define TIM24_PCLK_DIS() (RCC->APB1HENR &= ~(1 << 25))

#define TIM1 ((GenTimer_RegDef_t *)TIM1_BASEADDR)
#define TIM2 ((GenTimer_RegDef_t *)TIM2_BASEADDR)
#define TIM3 ((GenTimer_RegDef_t *)TIM3_BASEADDR)
#define TIM4 ((GenTimer_RegDef_t *)TIM4_BASEADDR)
#define TIM5 ((GenTimer_RegDef_t *)TIM5_BASEADDR)
#define TIM6 ((GenTimer_RegDef_t *)TIM6_BASEADDR)
#define TIM7 ((GenTimer_RegDef_t *)TIM7_BASEADDR)
#define TIM8 ((GenTimer_RegDef_t *)TIM8_BASEADDR)
#define TIM12 ((GenTimer_RegDef_t *)TIM12_BASEADDR)
#define TIM13 ((GenTimer_RegDef_t *)TIM13_BASEADDR)
#define TIM14 ((GenTimer_RegDef_t *)TIM14_BASEADDR)
#define TIM16 ((GenTimer_RegDef_t *)TIM15_BASEADDR)
#define TIM15 ((GenTimer_RegDef_t *)TIM16_BASEADDR)
#define TIM17 ((GenTimer_RegDef_t *)TIM17_BASEADDR)
#define TIM23 ((GenTimer_RegDef_t *)TIM23_BASEADDR)
#define TIM24 ((GenTimer_RegDef_t *)TIM24_BASEADDR)

typedef enum {
  TIM_CR1_CEN = 0,
  TIM_CR1_UDIS,
  TIM_CR1_URS,
  TIM_CR1_OPM,
  TIM_CR1_DIR,
  TIM_CR1_CMS,
  TIM_CR1_ARPE = 7,
  TIM_CR1_CKD,
  TIM_CR1_UIFRE_MAP = 11
} TIM_CR1_BIT; // Control Register 1

typedef enum {
  TIM_CR2_CCPC = 0,
  TIM_CR2_CCUS = 2,
  TIM_CR2_CCDS,
  TIM_CR2_MMS,
  TIM_CR2_TI1S = 7,
  TIM_CR2_OIS1,
  TIM_CR2_OIS1N,
  TIM_CR2_OIS2,
  TIM_CR2_OIS2N,
  TIM_CR2_OIS3,
  TIM_CR2_OIS3N,
  TIM_CR2_OIS4,
  TIM_CR2_OIS5 = 16,
  TIM_CR2_OIS6 = 18,
  TIM_CR2_MMS2 = 20
} TIM_CR2_BIT; // Control Register 2

typedef enum {
  TIM_SMCR_SMS = 0,
  TIM_SMCR_TS = 4,
  TIM_SMCR_MSM = 7,
  TIM_SMCR_ETF = 9,
  TIM_SMCR_ETPS = 12,
  TIM_SMCR_ECE = 14,
  TIM_SMCR_ETP = 15,
  TIM_SMCR_SMS2 = 16,
  TIM_SMCR_TS2 = 20,
} TIM_SMCR_BIT; // Slave Mode Control Register

typedef enum {
  TIM_DIER_UIE = 0,
  TIM_DIER_CC1IE,
  TIM_DIER_CC2IE,
  TIM_DIER_ССЗІЕ,
  TIM_DIER_CC4IE,
  TIM_DIER_COMIE,
  TIM_DIER_TIE,
  TIM_DIER_BIE,
  TIM_DIER_UDE,
  TIM_DIER_CC1DE,
  TIM_DIER_CC2DE,
  TIM_DIER_CC3DE,
  TIM_DIER_CC4DE,
  TIM_DIER_COMDE,
  TIM_DIER_TDE
} TIM_DIER_BIT; // DMA/Interrupt Enable Register

typedef enum {
  TIM_SR_UIF = 0,
  TIM_SR_CC1IF,
  TIM_SR_CC2IF,
  TIM_SR_CC3IF,
  TIM_SR_CC4IF,
  TIM_SR_COMIF,
  TIM_SR_TIF,
  TIM_SR_BIF,
  TIM_SR_B2IF,
  TIM_SR_CC10F,
  TIM_SR_CC2OF,
  TIM_SR_CC3OF,
  TIM_SR_CC4OF,
  TIM_SR_SBIF,
  TIM_SR_CC5IF = 16,
  TIM_SR_CC6IF
} TIM_SR_BIT; // Status Register

typedef enum {
  TIM_EGR_UG = 0,
  TIM_EGR_CC1G,
  TIM_EGR_CC2G,
  TIM_EGR_CC3G,
  TIM_EGR_CC4G,
  TIM_EGR_COMG,
  TIM_EGR_TG,
  TIM_EGR_BG,
  TIM_EGR_B2G
} TIM_EGR_BIT; // Event Generation Register

typedef enum {
  TIM_CCMR1_CC1S = 0,
  TIM_CCMR1_OC1_FE = 2,
  TIM_CCMR1_OC1_PE,
  TIM_CCMR1_OC1M,
  TIM_CCMR1_OC1_CE = 7,
  TIM_CCMR1_CC2S,
  TIM_CCMR1_OC2_FE = 10,
  TIM_CCMR1_OC2_PE,
  TIM_CCMR1_OC2M,
  TIM_CCMR1_OC2_CE = 15,
  TIM_CCMR1_OC1M2,
  TIM_CCMR1_OC2M2 = 24
} TIM_CCMR1_BIT; // Capture/Compare Mode Register 1

typedef enum {
  TIM_CCMR2_OP_CC1S = 0,
  TIM_CCMR2_OP_IC1PSC = 2,
  TIM_CCMR2_OP_IC1F = 4,
  TIM_CCMR2_OP_CC2S = 8,
  TIM_CCMR2_OP_IC2PSC = 10,
  TIM_CCMR2_OP_IC2F = 12,
} TIM_CCMR2_OP_BIT; // Capture/Compare Mode Register 2

typedef enum {
  TIM_CCMR2_IP_CC3S = 0,
  TIM_CCMR2_IP_OC3_FE = 2,
  TIM_CCMR2_IP_OC3_PE,
  TIM_CCMR2_IP_OC3M,
  TIM_CCMR2_IP_OC3_CE = 7,
  TIM_CCMR2_IP_CC4S,
  TIM_CCMR2_IP_OC4_FE = 10,
  TIM_CCMR2_IP_OC4_PE,
  TIM_CCMR2_IP_OC4M,
  TIM_CCMR2_IP_OC4_CE = 15,
  TIM_CCMR2_IP_OC3M2,
  TIM_CCMR2_IP_OC4M2 = 24
} TIM_CCER_IP_BIT; // Capture/Compare Enable Register

typedef enum {
  TIM_CCER_CC1E = 0,
  TIM_CCER_CC1P,
  TIM_CCER_CC1NE,
  TIM_CCER_CC1NP,
  TIM_CCER_CC2E,
  TIM_CCER_CC2P,
  TIM_CCER_CC2NE,
  TIM_CCER_CC2NP,
  TIM_CCER_CC3E,
  TIM_CCER_CC3P,
  TIM_CCER_CC3NE,
  TIM_CCER_CC3NP,
  TIM_CCER_CC4E,
  TIM_CCER_CC4P,
  TIM_CCER_CC4NP = 15,
  TIM_CCER_CC5E,
  TIM_CCER_CC5P,
  TIM_CCER_CC6E = 20,
  TIM_CCER_CC6P
} TIM_CCER_BIT; // Capture/Compare Enable Register

typedef enum { TIM_CNT = 0, TIM_CNT_UIF_CPY = 31 } TIM_CNT_BIT; // Counter

typedef enum { TIM_PSC = 0 } TIM_PSC_BIT; // Prescaler

typedef enum { TIM_ARR = 0 } TIM_ARR_BIT; // Auto-Reload Register

typedef enum { TIM_RCR_REP = 0 } TIM_RCR_BIT; // Repetition Counter Register

typedef enum { TIM_CCR1 = 0 } TIM_CCR1_BIT; // Capture/Compare Register 1

typedef enum { TIM_CCR2 = 0 } TIM_CCR2_BIT; // Capture/Compare Register 2

typedef enum { TIM_CCR3 = 0 } TIM_CCR3_BIT; // Capture/Compare Register 3

typedef enum { TIM_CCR4 = 0 } TIM_CCR4_BIT; // Capture/Compare Register 4

typedef enum {
  TIM_BDTR_DTG = 0,
  TIM_BDTR_LOCK = 8,
  TIM_BDTR_OSSI = 10,
  TIM_BDTR_OSSR,
  TIM_BDTR_BKE,
  TIM_BDTR_BKP,
  TIM_BDTR_AOE,
  TIM_BDTR_MOE,
  TIM_BDTR_BKF,
  TIM_BDTR_BK2F = 20,
  TIM_BDTR_BK2E = 24,
  TIM_BDTR_BK2P,
  TIM_BDTR_BKDSRM,
  TIM_BDTR_BK2DSRM,
  TIM_BDTR_BKBID,
  TIM_BDTR_BK2BID
} TIM_BDTR_BIT; // Break and Dead-Time Register

typedef enum {
  TIM_DCR_DBA = 0,
  TIM_DCR_DBL = 8
} TIM_DCR_BIT; // DMA Control Register

typedef enum {
  TIM_DMAR_DMAB = 0
} TIM_DMAR_BIT; // DMA Address for Full Transfer

typedef enum {
  TIM_CCMR3_OC5FE = 2,
  TIM_CCMR3_OC5PE,
  TIM_CCMR3_OC5M,
  TIM_CCMR3_OC5CE = 7,
  TIM_CCMR3_OC6FE = 10,
  TIM_CCMR3_OC6PE,
  TIM_CCMR3_OC6M,
  TIM_CCMR3_OC6CE = 15,
  TIM_CCMR3_OC5M3,
  TIM_CCMR3_OC6M3 = 24,
} TIM_CCMR3_BIT; // Capture/Compare Mode Register 3

typedef enum {
  TIM_CCR5 = 0,
  TIM_CCR5_GC5C1 = 29,
  TIM_CCR5_GC5C2,
  TIM_CCR5_GC5C3
} TIM_CCR5_BIT; // Capture/Compare Register 5

typedef enum { TIM_CCR6 = 0 } TIM_CCR6_BIT; // Capture/Compare Register 6

typedef enum {
  TIM_AF1_BKINE = 0,
  TIM_AF1_BKCMP1E,
  TIM_AF1_BKCMP2E,
  TIM_AF1_BKDF1BK0E = 8,
  TIM_AF1_BKINP,
  TIM_AF1_BKCMP1P,
  TIM_AF1_BKCMP2P,
  TIM_AF1_ETRSEL = 14,
} TIM_AF1_BIT; // Alternate Function Option Register 1

typedef enum {
  TIM_AF2_BK2INE = 0,
  TIM_AF2_BK2CMP1E,
  TIM_AF2_BK2CMP2E,
  TIM_AF2_BK2DF1BK3E = 8,
  TIM_AF2_BK2INP,
  TIM_AF2_BK2CMP1P,
  TIM_AF2_BK2CMP2P
} TIM_AF2_BIT; // Alternate Function Option Register 2

typedef enum {
  TI1SEL = 0,
  TI2SEL = 8,
  TI3SEL = 16,
  TI4SEL = 24,
} TIM_TISEL_BIT; // Timer Input Selection Register

/************** END: Timer register bit definitions ************/

#endif /* STM32H743ZG_PERIPHERALS_H_ */
