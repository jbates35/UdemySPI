/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#define FAST 100000
#define MEDIUM 300000
#define SLOW 1000000
#define WAIT(CNT)                                                              \
  do {                                                                         \
    for (int def_i = 0; def_i < CNT; def_i++)                                  \
      ;                                                                        \
  } while (0)

#include <stdint.h>
#include <string.h>

#include "stm32h723xx.h"
#include "stm32h723xx_gpio.h"
#include "stm32h723xx_spi.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning                                                                       \
    "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void program_init(void);

int main(void) {
  program_init();

  char user_data[] = "Hello world";
  int len = strlen(user_data);

  /* Loop forever */
  for (;;) {
    //    SPI_send(SPI1, (uint8_t *)user_data, len);
    //    WAIT(SLOW);
  }
}

void program_init(void) {
  // User button PC13
  // LED is on PB0
  GPIO_peri_clock_control(GPIOA, ENABLE);
  GPIO_peri_clock_control(GPIOB, ENABLE);
  GPIO_peri_clock_control(GPIOC, ENABLE);
  GPIO_peri_clock_control(GPIOE, ENABLE);
  SPI_peri_clock_control(SPI1, ENABLE);

  SYSCFG_PCLK_EN();

  // Create GPIO handler and assign shortcuts for easier access
  GPIO_Handle_t gpio_handle;
  GPIO_RegDef_t **addr = &gpio_handle.p_GPIO_x;
  GPIO_PinConfig_t *cfg = &gpio_handle.GPIO_pin_config;

  *addr = GPIOA;
  // PE4 - SPI_4_NSS
  cfg->GPIO_pin_number = 4;
  cfg->GPIO_pin_mode = GPIO_MODE_ALTFN;
  cfg->GPIO_pin_speed = GPIO_SPEED_HIGH;
  cfg->GPIO_pin_pupd_control = GPIO_PUPDR_NONE;
  cfg->GPIO_pin_out_type = GPIO_OP_TYPE_OPENDRAIN;
  cfg->GPIO_pin_alt_func_mode = 5;
  GPIO_init(&gpio_handle);

  // PE5 - SPI_4_MISO
  cfg->GPIO_pin_number = 5;
  GPIO_init(&gpio_handle);

  // PE6 - SPI_4_MOSI
  cfg->GPIO_pin_number = 6;
  GPIO_init(&gpio_handle);

  // PE2 - SPI_4_SCK
  cfg->GPIO_pin_number = 2;
  GPIO_init(&gpio_handle);

  // User button PC13
  *addr = GPIOC;
  cfg->GPIO_pin_number = 13;
  cfg->GPIO_pin_mode = GPIO_MODE_IT_FT;
  cfg->GPIO_pin_speed = GPIO_SPEED_LOW;
  cfg->GPIO_pin_pupd_control = GPIO_PUPDR_PULLDOWN;
  cfg->GPIO_pin_out_type = 0;
  cfg->GPIO_pin_alt_func_mode = 0;
  GPIO_init(&gpio_handle);
  GPIO_irq_interrupt_config(IRQ_NO_EXTI15_10, ENABLE);
  GPIO_irq_priority_config(IRQ_NO_EXTI15_10, 13);

  // LED PB0
  *addr = GPIOE;
  cfg->GPIO_pin_number = 1;
  cfg->GPIO_pin_mode = GPIO_MODE_OUT;
  cfg->GPIO_pin_speed = GPIO_SPEED_LOW;
  cfg->GPIO_pin_pupd_control = GPIO_PUPDR_NONE;
  cfg->GPIO_pin_out_type = GPIO_OP_TYPE_PUSHPULL;
  cfg->GPIO_pin_alt_func_mode = 0;
  GPIO_init(&gpio_handle);
  /*
    // Config spi
    SPI_Handle_t spi_handle;
    SPI_Config_t *spi_cfg = &spi_handle.SPI_config;
    SPI_RegDef_t **spi_reg = &spi_handle.p_SPI_x;

    *spi_reg = SPI1;
    spi_cfg->device_mode = SPI_DEVICE_MODE_MASTER;
    spi_cfg->bus_config = SPI_BUS_CONFIG_HALF_DUPLEX;
    spi_cfg->cpol = SPI_CPOL_CAPTURE_ACTIVE_HIGH;
    spi_cfg->cpha = SPI_CPHA_CAPTURE_SECOND_EDGE;
    spi_cfg->ssm = SPI_SSM_ENABLE;
    spi_cfg->baud_divisor = SPI_BAUD_DIVISOR_8;
    spi_cfg->dff = SPI_DFF_16_BIT;
    SPI_init(&spi_handle);
  */
  int asdf2 = 0;
}

void EXTI15_10_IRQHandler(void) {
  if (GPIO_irq_handling(13))
    GPIO_toggle_output_pin(GPIOE, 1);
}
