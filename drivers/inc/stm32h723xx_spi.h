
/*
 * STM32H723xx_spi.h
 *
 *  Created on: Dec. 11, 2023
 *      Author: jbates
 */

#ifndef INC_STM32H723XX_SPI_H_
#define INC_STM32H723XX_SPI_H_

#include "stm32h723xx.h"

/** Enums used in SPI configuration **/
typedef enum SPI_DEVICE_MODE {
  SPI_DEVICE_MODE_SLAVE = 0,
  SPI_DEVICE_MODE_MASTER
} SPI_DEVICE_MODE;

// Bus configuration (simplex duplex etc)
typedef enum SPI_BUS_CONFIG {
  SPI_BUS_CONFIG_FULL_DUPLEX = 0,
  SPI_BUS_CONFIG_SIMPLEX_TX_ONLY,
  SPI_BUS_CONFIG_SIMPLEX_RX_ONLY,
  SPI_BUS_CONFIG_HALF_DUPLEX
} SPI_BUS_CONFIG;

// Data frame format
typedef enum SPI_DFF {
  SPI_DFF_4_BIT = 0x3,
  SPI_DFF_8_BIT = 0x7,
  SPI_DFF_16_BIT = 0xf,
  SPI_DFF_24_BIT = 0x17,
  SPI_DFF_32_BIT = 0x1f
} SPI_DFF;

// Clock phase angle
typedef enum SPI_CPHA {
  SPI_CPHA_CAPTURE_FIRST_EDGE = 0,
  SPI_CPHA_CAPTURE_SECOND_EDGE
} SPI_CPHA;

// Clock polarity
typedef enum SPI_CPOL {
  SPI_CPOL_CAPTURE_ACTIVE_HIGH = 0,
  SPI_CPOL_CAPTURE_ACTIVE_LOW
} SPI_CPOL;

// Software slave management
typedef enum SPI_SSM { SPI_SSM_DISABLE = 0, SPI_SSM_ENABLE } SPI_SSM;

typedef enum SPI_BAUD_DIVISOR {
  SPI_BAUD_DIVISOR_2 = 0,
  SPI_BAUD_DIVISOR_4,
  SPI_BAUD_DIVISOR_8,
  SPI_BAUD_DIVISOR_16,
  SPI_BAUD_DIVISOR_32,
  SPI_BAUD_DIVISOR_64,
  SPI_BAUD_DIVISOR_128,
  SPI_BAUD_DIVISOR_256
} SPI_BAUD_DIVISOR;

/**
 * DeviceMode = SPI master, spi slave, etc.
 * BusConfig = Full duplex, half duplex, simplex
 * DFF - Data Frame Format (8bit data vs 16bit data)
 * CPHA - clock phase
 * CPOL - clock polarity
 * SSM - slave select management, software vs hardware
 * Speed - SPI clock speed based on divisors
 * */
typedef struct SPI_Config {
  SPI_DEVICE_MODE device_mode;
  SPI_BUS_CONFIG bus_config;
  SPI_DFF dff;
  SPI_CPHA cpha;
  SPI_CPOL cpol;
  SPI_SSM ssm;
  SPI_BAUD_DIVISOR baud_divisor;
} SPI_Config_t;

typedef struct SPI_Handle {
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
void SPI_send(SPI_RegDef_t *p_SPI_x, uint8_t *p_tx_buffer, int len);

/**
 * SPI function for receiving data
 **/
void SPI_receive(SPI_RegDef_t *p_SPI_x, uint8_t *p_rx_buffer, uint32_t len);

/**
 * IRQ Configuration and ISR handling
 * */
#endif
