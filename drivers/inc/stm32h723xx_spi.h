
/*
 * STM32H723xx_spi.h
 *
 *  Created on: Dec. 11, 2023
 *      Author: jbates
 */

#ifndef INC_STM32H723XX_SPI_H_
#define INC_STM32H723XX_SPI_H_

#include "stm32h723xx.h"

typedef struct {
  __vo uint32_t CR1;
  __vo uint32_t CR2;
  __vo uint32_t CFG1;
  __vo uint32_t CFG2;
  __vo uint32_t IER;
  __vo uint32_t SR;
  __vo uint32_t IFCR;
  __vo uint32_t TXDR;
  uint32_t RESERVED0[3];
  __vo uint32_t RXDR;
  uint32_t RESERVED1[3];
  __vo uint32_t CRCPOLY;
  __vo uint32_t TXCRC;
  __vo uint32_t RXCRC;
  __vo uint32_t UDRDR;
  __vo uint32_t SPI_I2SCFGR;

} SPI_Config_t;

typedef struct {
  GPIO_RegDef_t *
      p_GPIO_x; // Holds the base address of the GPIO port which the pin belongs
  GPIO_PinConfig_t GPIO_pin_config; // Holds the GPIO pin configuration settings
} GPIO_Handle_t;

#endif
