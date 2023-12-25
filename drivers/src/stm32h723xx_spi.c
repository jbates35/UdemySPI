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

  // Might want to reset the entre SPI bus here

  (*spi_reg)->CFG2 = cfg->device_mode << 22;
  (*spi_reg)->CFG2 |= cfg->bus_config << 17;
  (*spi_reg)->CFG2 |= cfg->cpha << 24;
  (*spi_reg)->CFG2 |= cfg->cpol << 25;

  // Not sure I need the outer if statement. Only allows for SSM when in master
  // mode.
  if (cfg->device_mode == SPI_MASTER) {
    if (cfg->ssm == SPI_SSM_ENABLE)
      (*spi_reg)->CFG2 |= (1 << 26);
    else
      (*spi_reg)->CFG2 |= (0 << 26) + (1 << 29);
  }

  (*spi_reg)->CFG1 = (cfg->speed << 28);
  (*spi_reg)->CFG1 |= (cfg->dff << 0);

  // Lock the SPI registers so SPI is ready for use
  (*spi_reg)->CR1 |= (1 << 16) + (1 << 0);
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
