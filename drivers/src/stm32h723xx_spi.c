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
  (*spi_reg)->CR1 &= ~((1 << SPI_CR1_IOLOCK) + (1 << SPI_CR1_SPE));

  // Reset the SPI peripheral before configuring
  static SPI_RegDef_t *const SPIx_BASE_ADDRS[6] = {SPI1, SPI2, SPI3,
                                                   SPI4, SPI5, SPI6};

  static uint8_t const SPIx_EN_BIT_POS[6] = {12, 14, 15, 13, 20, 5};

  uint32_t SPIx_RCC_REG[6] = {RCC->APB2RSTR, RCC->APB1LRSTR, RCC->APB1LRSTR,
                              RCC->APB2RSTR, RCC->APB2RSTR,  RCC->APB4RSTR};

  for (int i = 0; i < sizeof(SPIx_BASE_ADDRS) / sizeof(SPI_RegDef_t *); i++) {
    if (*spi_reg != SPIx_BASE_ADDRS[i])
      continue;

    SPIx_RCC_REG[i] |= (1 << SPIx_EN_BIT_POS[i]);
    SPIx_RCC_REG[i] &= ~(1 << SPIx_EN_BIT_POS[i]);

    break;
  }

  // Set parameters given to us from the config struct
  (*spi_reg)->CFG2 = cfg->device_mode << SPI_CFG2_MASTER;
  (*spi_reg)->CFG2 |= cfg->bus_config << SPI_CFG2_COMM;
  (*spi_reg)->CFG2 |= cfg->cpha << SPI_CFG2_CPHA;
  (*spi_reg)->CFG2 |= cfg->cpol << SPI_CFG2_CPOL;

  // Not sure I need the outer if statement. Only allows for SSM when in master
  // mode.
  if (cfg->device_mode == SPI_MASTER) {
    if (cfg->ssm == SPI_SSM_ENABLE)
      (*spi_reg)->CFG2 |= (1 << SPI_CFG2_SSM);
    else
      (*spi_reg)->CFG2 |= (0 << SPI_CFG2_SSM) + (1 << SPI_CFG2_SSOE);
  }

  // Set the baud rate and data frame size
  (*spi_reg)->CFG1 = (cfg->speed << SPI_CFG1_MBR);
  (*spi_reg)->CFG1 |= (cfg->dff << SPI_CFG1_DSIZE);

  // Lock the SPI registers so SPI is ready for use
  (*spi_reg)->CR1 |= (1 << SPI_CR1_IOLOCK) + (1 << SPI_CR1_SPE);
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
