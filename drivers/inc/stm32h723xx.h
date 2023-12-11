/*
 * STM32H743ZG_peripherals.h
 *
 *  Created on: Sep 29, 2023
 *      Author: jbates
 */

#ifndef STM32H723ZG_PERIPHERALS_H_
#define STM32H723ZG_PERIPHERALS_H_

#include <stdint.h>

#define __vo volatile //For easy casting as volatile memory

/************** START: Processor specific details (cortex-m7) *****************/
/**
 * ARM Cortex M7 processor NVIC ISERx and ICERx register addresses
*/
#define NVIC_ISERX_BASEADDR         0xE000E100
#define NVIC_ICERX_BASEADDR         0xE000E180 

typedef struct NVIC_RegDef {
    __vo uint32_t ISER[8];          // NVIC Interrupt set registers
    uint32_t RESERVED1[24];
    __vo uint32_t ICER[8];          // NVIC Interrupt clear registers
    uint32_t RESERVED2[24];
    __vo uint32_t ISPR[8];          // NVIC Interrupt set pending registers
    uint32_t RESERVED3[24];
    __vo uint32_t ICPR[8];          // NVIC Interrupt clear pending registers
    uint32_t RESERVED4[24];
    __vo uint32_t IABR[8];          // NVIC Interrupt active bit register (read only)
    uint32_t RESERVED5[56];
    __vo uint32_t IPR[60];          // NVIC Interrupt priority register
} NVIC_RegDef_t;

#define NVIC                        ((NVIC_RegDef_t*) NVIC_ISERX_BASEADDR)


/************** END: Processor specific details (cortex-m7) *****************/

//Base addresses of peripherals
#define HSEM_BASEADDR			    0x58026400
#define ADC3_BASEADDR               0x58026000
#define DMAMUX2_BASEADDR            0x58025800
#define BDMA_BASEADDR               0x58025400
#define CRC_BASEADDR                0x58024C00
#define PWR_BASEADDR                0x58024800
#define RCC_BASEADDR                0x58024400

#define GPIOK_BASEADDR              0x58022800
#define GPIOJ_BASEADDR              0x58022400
#define GPIOH_BASEADDR              0x58021C00
#define GPIOG_BASEADDR              0x58021800
#define GPIOF_BASEADDR              0x58021400
#define GPIOE_BASEADDR              0x58021000
#define GPIOD_BASEADDR              0x58020C00
#define GPIOC_BASEADDR              0x58020800
#define GPIOB_BASEADDR              0x58020400
#define GPIOA_BASEADDR              0x58020000

#define DTS_BASEADDR                0x58006800   
#define SAI4_BASEADDR               0x58005400
#define IWDG_BASEADDR               0x58004800     
#define RTC_BASEADDR                0x58004000     
#define VREF_BASEADDR               0x58003C00     
#define COMP1_BASEADDR              0x58003800     
#define LPTIM5_BASEADDR             0x58003000     
#define LPTIM4_BASEADDR             0x58002C00     
#define LPTIM3_BASEADDR             0x58002800     
#define LPTIM2_BASEADDR             0x58002400     
#define I2C4_BASEADDR               0x58001C00     
#define SPI_I2S6_BASEADDR           0x58001400     
#define LPUART1_BASEADDR            0x58000C00         
#define SYSCFG_BASEADDR             0x58000400     
#define EXTI_BASEADDR               0x58000000    
 
#define OTFDEC2_BASEADDR            0x5200BC00
#define OTFDEC1_BASEADDR            0x5200B800
#define OCTOSPI_IO_BASEADDR         0x5200B400
#define OCTOSPI2_DELAY_BASEADDR     0x5200B000
#define OCTOSPI2_BASEADDR           0x5200A000
#define RAMECC1_BASEADDR            0x52009000         
#define SDMMC1_DELAY_BASEADDR       0x52008000             
#define SDMMC1_BASEADDR             0x52007000  
#define OCTOSPI1_DELAY_BASEADDR     0x52006000             
#define OCTOSPI1_CTRL_BASEADDR      0x52005000             
#define FMC_CTRL_BASEADDR           0x52004000         
#define FLASH_INTERFACE_BASEADDR    0x52002000                  
#define DMA2D_CHROM_ART_BASEADDR    0x52001000                 
#define MDMA_BASEADDR               0x52000000     
#define GPV_BASEADDR                0x51000000     
#define WWDG_BASEADDR               0x50003000     
#define LTDC_BASEADDR               0x50001000  
#define CORDIC_BASEADDR             0x48024400
#define FMAC_BASEADDR               0x48024000
#define RAMECC_BASEADDR             0x48023000         
#define SDMMC2_DELAY_BASEADDR       0x48022800 
#define SDMMC2_BASEADDR             0x48022400  
#define RNG_BASEADDR                0x48021800  
#define HASH_BASEADDR               0x48021400  
#define CRYPTO_BASEADDR             0x48021000  
#define PSSI_BASEADDR               0x48020400
#define DCMI_BASEADDR               0x48020000

#define USB1_OTG_HS_BASEADDR        0x40040000  
#define ETHERNET_MAC_BASEADDR       0x40028000  
#define ADC1_ADC2_BASEADDR          0x40022000  
#define DMAMUX1_BASEADDR            0x40020800  
#define DMA2_BASEADDR               0x40020400  
#define DMA1_BASEADDR               0x40020000  
#define DFSDM1_BASEADDR             0x40017800
#define SAI1_BASEADDR               0x40015800  
#define SPI5_BASEADDR               0x40015000  
#define TIM17_BASEADDR              0x40014800  
#define TIM16_BASEADDR              0x40014400  
#define TIM15_BASEADDR              0x40014000  
#define SPI4_BASEADDR               0x40013400  
#define SPI1_I2S1_BASEADDR          0x40013000
#define USART10_BASEADDR            0x40011C00  
#define UART9_BASEADDR              0x40011800  
#define USART6_BASEADDR             0x40011400  
#define USART1_BASEADDR             0x40011000  
#define TIM8_BASEADDR               0x40010400  
#define TIM1_BASEADDR               0x40010000  

#define TIM24_BASEADDR              0x4000E400
#define TIM23_BASEADDR              0x4000E000 
#define FDCAN3_BASEADDR             0x4000D400 
#define CAN_BASEADDR                0x4000AC00 
#define CAN_CCU_BASEADDR            0x4000A800 
#define FDCAN2_BASEADDR             0x4000A400 
#define FDCAN1_BASEADDR             0x4000A000 
#define MDIOS_BASEADDR              0x40009400 
#define OPAMP_BASEADDR              0x40009000 
#define SWPMI_BASEADDR              0x40008800 
#define CRS_BASEADDR                0x40008400 
#define UART8_BASEADDR              0x40007C00 
#define UART7_BASEADDR              0x40007800 
#define DAC1_BASEADDR               0x40007400 
#define HDMI_CEC_BASEADDR           0x40006C00
#define I2C5_BASEADDR               0x40006400
#define I2C3_BASEADDR               0x40005C00 
#define I2C2_BASEADDR               0x40005800 
#define I2C1_BASEADDR               0x40005400 
#define UART5_BASEADDR              0x40005000 
#define UART4_BASEADDR              0x40004C00 
#define USART3_BASEADDR             0x40004800 
#define USART2_BASEADDR             0x40004400 
#define SPDIFRX1_BASEADDR           0x40004000 
#define SPI3_I2S3_BASEADDR          0x40003C00 
#define SPI2_I2S2_BASEADDR          0x40003800 
#define LPTIM1_BASEADDR             0x40002400 
#define TIM14_BASEADDR              0x40002000 
#define TIM13_BASEADDR              0x40001C00 
#define TIM12_BASEADDR              0x40001800 

#define TIM7_BASEADDR               0x40001400 
#define TIM6_BASEADDR               0x40001000 
#define TIM5_BASEADDR               0x40000C00 
#define TIM4_BASEADDR               0x40000800 
#define TIM3_BASEADDR               0x40000400 
#define TIM2_BASEADDR               0x40000000 


// -------------peripheral structs-------------
// Use these with and declare them as pointer structs to 
// be able to access the memory easily

typedef struct RCC_RegDef {
    __vo uint32_t CR;          //0
    __vo uint32_t HSICFGR;
    __vo uint32_t CRRCR;
    __vo uint32_t CSICFGR;     
    __vo uint32_t CFGR;        //10
    uint32_t RESERVED1;
    __vo uint32_t D1CFGR; //good
    __vo uint32_t D2CFGR;      
    __vo uint32_t D3CFGR;      //20
    uint32_t RESERVED2;
    __vo uint32_t PLLCKSELR;
    __vo uint32_t PLLCFGR;     
    __vo uint32_t PLL1DIVR;    //30
    __vo uint32_t PLL1FRACR;
    __vo uint32_t PLL2DIVR;
    __vo uint32_t PLL2FRACR;   
    __vo uint32_t PLL3DIVR;    //40
    __vo uint32_t PLL3FRACR;
    uint32_t RESERVED3;
    __vo uint32_t D1CCIPR; //good 
    __vo uint32_t D2CCIP1R;    //50
    __vo uint32_t D2CCIP2R;
    __vo uint32_t D3CCIPR;
    uint32_t RESERVED4;    
    __vo uint32_t CIER;        //60
    __vo uint32_t CIFR;
    __vo uint32_t CICR;
    uint32_t RESERVED5;    
    __vo uint32_t BDCR; //good //70
    __vo uint32_t CSR;
    uint32_t RESERVED6; 
    __vo uint32_t AHB3RSTR;    
    __vo uint32_t AHB1RSTR;    //80
    __vo uint32_t AHB2RSTR;
    __vo uint32_t AHB4RSTR;
    __vo uint32_t APB3RSTR;    
    __vo uint32_t APB1LRSTR;   //90
    __vo uint32_t APB1HRSTR; //good
    __vo uint32_t APB2RSTR;
    __vo uint32_t APB4RSTR;    
    __vo uint32_t GCR;         //A0
    uint32_t RESERVED7; 
    __vo uint32_t D3AMR;
    uint32_t RESERVED8[9];    
    __vo uint32_t RSR;         //D0
    __vo uint32_t AHB3ENR;
    __vo uint32_t AHB1ENR; //good
    __vo uint32_t AHB2ENR;     
    __vo uint32_t AHB4ENR;     //E0
    __vo uint32_t APB3ENR;
    __vo uint32_t APB1LENR;
    __vo uint32_t APB1HENR;      
    __vo uint32_t APB2ENR;     //F0 
    __vo uint32_t APB4ENR;
    uint32_t RESERVED9; 
    __vo uint32_t AHB3LPENR; 
    __vo uint32_t AHB1LPENR;   // good 00
    __vo uint32_t AHB2LPENR;
    __vo uint32_t AHB4LPENR;
    __vo uint32_t APB3LPENR;   
    __vo uint32_t APB1LLPENR;  //10
    __vo uint32_t APB1HLPENR;
    __vo uint32_t APB2LPENR; // good
    __vo uint32_t APB4LPENR;   
    uint32_t RESERVED10[4];    
    __vo uint32_t C1_RSR;       //30
    __vo uint32_t C1_AHB3ENR;
    __vo uint32_t C1_AHB1ENR; 
    __vo uint32_t C1_AHB2ENR;   
    __vo uint32_t C1_AHB4ENR;   //40
    __vo uint32_t C1_APB3ENR; // good
    __vo uint32_t C1_APB1LENR; 
    __vo uint32_t C1_APB1HENR; 
    __vo uint32_t C1_APB2ENR;   //50
    __vo uint32_t C1_APB4ENR;
    uint32_t RESERVED11;    
    __vo uint32_t C1_AHB3LPENR;    
    __vo uint32_t C1_AHB1LPENR;    //60
    __vo uint32_t C1_AHB2LPENR;
    __vo uint32_t C1_AHB4LPENR;    
    __vo uint32_t C1_APB3LPENR;    
    __vo uint32_t C1_APB1LLPENR;   //70
    __vo uint32_t C1_APB1HLPENR;
    __vo uint32_t C1_APB2LPENR;    
    __vo uint32_t C1_APB4LPENR;   
} RCC_RegDef_t;

typedef struct HSEM_RegDef {
    __vo uint32_t HSEM_R[32];    // HSEM register semaphore
    __vo uint32_t HSEM_RLR[32];  // HSEM read lock register semaphore
    __vo uint32_t HSEM_IER;       // HSEM interrupt enable register
    __vo uint32_t HSEM_ICR;       // HSEM interrupt clear register
    __vo uint32_t HSEM_ISR;       // HSEM interrupt status register
    __vo uint32_t HSEM_MISR;      // HSEM master interrupt status register
    __vo uint32_t HSEM_CR;        // HSEM clear register
    __vo uint32_t HSEM_KEYR;      // HSEM interrupt clear register
} HSEM_RegDef_t;

typedef struct GPIO_RegDef {
	__vo uint32_t MODER;        // GPIO Mode register
	__vo uint32_t OTYPER;       // GPIO Output type register
	__vo uint32_t OSPEEDR;      // GPIO Output speed register
	__vo uint32_t PUPDR;        // GPIO Pull-up/Pull-down register
	__vo uint32_t IDR;          // GPIO Input data register
	__vo uint32_t ODR;          // GPIO Output data register
	__vo uint32_t BSRR;         // GPIO Bit set/reset register
	__vo uint32_t LCKR;         // GPIO Port configuration lock register
	__vo uint32_t AFR[2];       // GPIO Alternate function registers
} GPIO_RegDef_t;

typedef struct SYSCFG_RegDef {
    uint32_t RESERVED1;
    __vo uint32_t PMCR;         // SysConfig peripheral mode configuration register
    __vo uint32_t EXTICR[4];    // SysConfig external interrupt configuration registers
    uint32_t RESERVED2[2];      // Reserved
	__vo uint32_t CCCSR;        // SysConfig compensation cell control/status register
    __vo uint32_t CCVR;         // SysConfig compensation cell value register
    __vo uint32_t CCCR;         // SysConfig compensation cell code register
    uint32_t RESERVED3;			// Reserved
	__vo uint32_t ADC2ALT;      // SysConfig internal input alternate connection register
    uint32_t RESERVED4[60];  	// Reserved
    __vo uint32_t PKGR;         // SysConfig package register
    uint32_t RESERVED5[118];		// Reserved
    __vo uint32_t UR[18];       // SysConfig user registers
} SYSCFG_RegDef_t;

typedef struct EXTI_RegDef { 
    __vo uint32_t RTSR1;       	// EXTI Rising trigger selection register
    __vo uint32_t FTSR1;       	// EXTI Falling trigger selection register
    __vo uint32_t SWIER1;      	// EXTI Software interrupt event register
    __vo uint32_t D3PMR1;      	// EXTI D3 Pending mask register
    __vo uint32_t D3PCR1L;     	// EXTI D3 Pending clear selection register low
    __vo uint32_t D3PCR1H;     	// EXTI D3 Pending clear selection register high
    uint32_t RESERVED1[2];		// Reserved
	__vo uint32_t RTSR2;       	// EXTI Rising trigger selection register
    __vo uint32_t FTSR2;       	// EXTI Falling trigger selection register
    __vo uint32_t SWIER2;      	// EXTI Software interrupt event register
    __vo uint32_t D3PMR2;      	// EXTI D3 Pending mask register
    __vo uint32_t D3PCR2L;     	// EXTI D3 Pending clear selection register low
    __vo uint32_t D3PCR2H;     	// EXTI D3 Pending clear selection register high
    uint32_t RESERVED2[2];     	// Reserved
    __vo uint32_t RTSR3;       	// EXTI Rising trigger selection register
    __vo uint32_t FTSR3;       	// EXTI Falling trigger selection register
    __vo uint32_t SWIER3;      	// EXTI Software interrupt event register
    __vo uint32_t D3PMR3;      	// EXTI D3 Pending mask register
    __vo uint32_t D3PCR3L;     	// EXTI D3 Pending clear selection register low
    __vo uint32_t D3PCR3H;     	// EXTI D3 Pending clear selection register high
    uint32_t RESERVED3[10];    	// Reserved
    __vo uint32_t CPUIMR1;     	// EXTI Interrupt mask register
    __vo uint32_t CPUCPUE_MR1; 	// EXTI Interrupt clear register
    __vo uint32_t CPUPR1;      	// EXTI Interrupt pending register
    uint32_t RESERVED4;			// Reserved
	__vo uint32_t CPUIMR2;     	// EXTI Interrupt mask register
    __vo uint32_t CPUEMR2;     	// EXTI Event mask register
    __vo uint32_t CPUPR2;      	// EXTI Pending register
    uint32_t RESERVED5;			// Reserved
	__vo uint32_t CPUIMR3;     	// EXTI Interrupt mask register
    __vo uint32_t CPUEMR3;     	// EXTI Event mask register
    __vo uint32_t CPUPR3;      	// EXTI Pending register
} EXTI_RegDef_t;

// Actual Pointer Notation with base addresses:
#define RCC                     ((RCC_RegDef_t*) RCC_BASEADDR)

#define HSEM_PCLK_EN()            (RCC->AHB4ENR |= (1 << 25))
#define GPIOA_PCLK_EN()           (RCC->AHB4ENR |= (1 << 0))
#define GPIOB_PCLK_EN()           (RCC->AHB4ENR |= (1 << 1))
#define GPIOC_PCLK_EN()           (RCC->AHB4ENR |= (1 << 2))
#define GPIOD_PCLK_EN()           (RCC->AHB4ENR |= (1 << 3))
#define GPIOE_PCLK_EN()           (RCC->AHB4ENR |= (1 << 4))
#define GPIOF_PCLK_EN()           (RCC->AHB4ENR |= (1 << 5))
#define GPIOG_PCLK_EN()           (RCC->AHB4ENR |= (1 << 6))
#define GPIOH_PCLK_EN()           (RCC->AHB4ENR |= (1 << 7))
#define GPIOJ_PCLK_EN()           (RCC->AHB4ENR |= (1 << 9))
#define GPIOK_PCLK_EN()           (RCC->AHB4ENR |= (1 << 10))

#define SYSCFG_PCLK_EN()          (RCC->APB4ENR |= (1 << 1))

#define HSEM                    ((HSEM_RegDef_t*) HSEM_BASEADDR)

#define GPIOA                   ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB                   ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC                   ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD                   ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE                   ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF                   ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG                   ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH                   ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOJ                   ((GPIO_RegDef_t*) GPIOJ_BASEADDR)
#define GPIOK                   ((GPIO_RegDef_t*) GPIOK_BASEADDR)

#define SYSCFG                  ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define EXTI                    ((EXTI_RegDef_t*) EXTI_BASEADDR)

//Generic macros
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

//IRQ macros
#define IRQ_NO_EXTI0        6
#define IRQ_NO_EXTI1        7
#define IRQ_NO_EXTI2        8
#define IRQ_NO_EXTI3        9
#define IRQ_NO_EXTI4        10
#define IRQ_NO_EXTI9_5      23
#define IRQ_NO_EXTI15_10    40


#endif /* STM32H743ZG_PERIPHERALS_H_ */
