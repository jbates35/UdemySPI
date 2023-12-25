
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

/** Enums used in SPI configuration **/
typedef enum { SPI_SLAVE = 0, SPI_MASTER } SPI_DEVICE_MODE;

typedef enum {
  SPI_FULL_DUPLEX = 0,
  SPI_SIMPLEX_TX_ONLY,
  SPI_SIMPLEX_RX_ONLY,
  SPI_HALF_DUPLEX
} SPI_BUS_CONFIG;

typedef enum { SPI_8_BIT = 0, SPI_16_BIT } SPI_DATA_SIZE;

typedef enum {
  SPI_CAPTURE_FIRST_EDGE = 0,
  SPI_CAPTURE_SECOND_EDGE
} SPI_CLOCK_PHASE;

typedef enum {
  SPI_CAPTURE_ACTIVE_HIGH = 0,
  SPI_CAPTURE_ACTIVE_LOW
} SPI_CLOCK_POLARITY;

typedef enum { SPI_SSM_DISABLE = 0, SPI_SSM_ENABLE } SPI_SSM_SOFTWARE_EN;

typedef enum {
  SPI_BAUD_RATE_2 = 0,
  SPI_BAUD_RATE_4,
  SPI_BAUD_RATE_8,
  SPI_BAUD_RATE_16,
  SPI_BAUD_RATE_32,
  SPI_BAUD_RATE_64,
  SPI_BAUD_RATE_128,
  SPI_BAUD_RATE_256
} SPI_BAUD_RATE;

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
