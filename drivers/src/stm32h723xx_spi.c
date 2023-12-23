#include "stm32h723xx_spi.h"

/**
 * Peripheral clock setup
 * */
void SPI_peri_clock_control(SPI_RegDef_t *p_gpio_x, uint8_t en_state) {}

/**
 * Init and de-init
 * */
void SPI_init(SPI_Handle_t *p_gpio_handle);
void SPI_deinit(SPI_RegDef_t *p_gpio_x);

/**
 * SPI function for sending data
 * */
void SPI_send(SPI_RegDef_t *p_SPI_x, uint8_t *p_tx_buffer, uint32_t len);

/**
 * SPI function for receiving data
 **/
void SPI_receive(SPI_RegDef_t *p_SPI_x, uint8_t *p_rx_buffer, uint32_t len);
