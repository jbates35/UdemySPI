
/*
 * STM32H723xx_spi.h
 *
 *  Created on: Dec. 11, 2023
 *      Author: jbates
 */

#ifndef INC_STM32H723XX_SPI_H_
#define INC_STM32H723XX_SPI_H_

#include "stm32h723xx.h"

/**
 * DeviceMode = SPI master, spi slave, etc.
 * BusConfig = Full duplex, half duplex, simplex
 * DFF - Data Frame Format (8bit data vs 16bit data)
 * CPHA - clock phase
 * CPOL - clock polarity
 * SSM - slave select management, software vs hardware
 * Speed - SPI clock speed based on divisors
 * */
typedef struct {
  uint8_t device_mode;
  uint8_t bus_config;
  uint8_t dff;
  uint8_t cpha;
  uint8_t cpol;
  uint8_t ssm;
  uint8_t speed;
} SPI_Config_t;

typedef struct {
  SPI_RegDef_t *p_SPI_x;
  SPI_Config_t SPI_config;
} SPI_Handle_t;

/**
 * Peripheral clock setup
 * */
void SPI_peri_clock_control(SPI_RegDef_t *p_SPI_x, uint8_t en_state);

/**
 * Init and de-init
 * */
void SPI_init(SPI_Handle_t *p_SPI_handle);
void SPI_deinit(SPI_RegDef_t *p_SPI_x);

/**
 * SPI function for sending data
 * */
void SPI_send(SPI_RegDef_t *p_SPI_x, uint8_t *p_tx_buffer, uint32_t len);

/**
 * SPI function for receiving data
 **/
void SPI_receive(SPI_RegDef_t *p_SPI_x, uint8_t *p_rx_buffer, uint32_t len);

/**
 * IRQ Configuration and ISR handling
 * */
#endif
