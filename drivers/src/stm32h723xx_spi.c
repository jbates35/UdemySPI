#include "stm32h723xx_spi.h"
#include "stm32h723xx.h"

#include <stdio.h>

/**
 * Peripheral clock setup
 * */
void SPI_peri_clock_control(SPI_RegDef_t *p_SPI_x, uint8_t en_state) {
  if (p_SPI_x == NULL)
    return;

  static SPI_RegDef_t *const SPIx_BASE_ADDRS[6] = {SPI1, SPI2, SPI3,
                                                   SPI4, SPI5, SPI6};

  static uint8_t const SPIx_EN_BIT_POS[6] = {12, 14, 15, 13, 20, 5};

  uint32_t SPIx_RCC_REG[6] = {RCC->APB2ENR, RCC->APB1LENR, RCC->APB1LENR,
                              RCC->APB2ENR, RCC->APB2ENR,  RCC->APB4ENR};

  for (int i = 0; i < sizeof(SPIx_BASE_ADDRS) / sizeof(SPI_RegDef_t *); i++) {
    if (p_SPI_x != SPIx_BASE_ADDRS[i])
      continue;

    if (en_state == ENABLE)
      SPIx_RCC_REG[i] |= (1 << SPIx_EN_BIT_POS[i]);
    else
      SPIx_RCC_REG[i] &= ~(1 << SPIx_EN_BIT_POS[i]);

    break;
  }
}

/**
 * Init and de-init
 * */
void SPI_init(SPI_Handle_t *p_SPI_handle) {
  if (p_SPI_handle == NULL)
    return;

  SPI_Config_t *cfg = &(p_SPI_handle->SPI_config);
  SPI_RegDef_t **spi_reg = &(p_SPI_handle->p_SPI_x);

  // First set IOLOCK bit to modify register
  (*spi_reg)->CR1 &= ~((1 << 16) + (1 << 0));

  // Set master or slave
  if (cfg->device_mode == (int)SPI_MASTER)
    (*spi_reg)->CFG2 |= (1 << 22);
  else
    (*spi_reg)->CFG2 &= ~(1 << 22);
}

void SPI_deinit(SPI_RegDef_t *p_SPI_x);

/**
 * SPI function for sending data
 * */
void SPI_send(SPI_RegDef_t *p_SPI_x, uint8_t *p_tx_buffer, uint32_t len);

/**
 * SPI function for receiving data
 **/
void SPI_receive(SPI_RegDef_t *p_SPI_x, uint8_t *p_rx_buffer, uint32_t len);
