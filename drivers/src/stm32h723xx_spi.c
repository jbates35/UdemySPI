#include "stm32h723xx_spi.h"
#include "stm32h723xx.h"

#include <stdio.h>

// Helper functions
void delay(uint32_t delay_ms) {
  volatile uint32_t i = 0;

  // TODO: Figure out clock cycles per ms
  for (i = 0; i < delay_ms * 1000; i++)
    ;
}

/**
 * Peripheral clock setup
 * */
void SPI_peri_clock_control(SPI_RegDef_t *p_SPI_x, uint8_t en_state) {
  if (p_SPI_x == NULL)
    return;

  static SPI_RegDef_t *const SPIx_BASE_ADDRS[6] = {SPI1, SPI2, SPI3,
                                                   SPI4, SPI5, SPI6};

  static uint8_t const SPIx_EN_BIT_POS[6] = {12, 14, 15, 13, 20, 5};

  volatile uint32_t *SPIx_RCC_REG[6] = {&(RCC->APB2ENR),  &(RCC->APB1LENR),
                                        &(RCC->APB1LENR), &(RCC->APB2ENR),
                                        &(RCC->APB2ENR),  &(RCC->APB4ENR)};

  for (int i = 0; i < sizeof(SPIx_BASE_ADDRS) / sizeof(SPI_RegDef_t *); i++) {
    if (p_SPI_x != SPIx_BASE_ADDRS[i])
      continue;

    if (en_state == ENABLE)
      *SPIx_RCC_REG[i] |= (1 << SPIx_EN_BIT_POS[i]);
    else
      *SPIx_RCC_REG[i] &= ~(1 << SPIx_EN_BIT_POS[i]);

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

  // Reset the SPI peripheral before configuring
  static SPI_RegDef_t *const SPIx_BASE_ADDRS[6] = {SPI1, SPI2, SPI3,
                                                   SPI4, SPI5, SPI6};

  static uint8_t const SPIx_EN_BIT_POS[6] = {12, 14, 15, 13, 20, 5};

  uint32_t SPIx_RCC_REG[6] = {RCC->APB2RSTR, RCC->APB1LRSTR, RCC->APB1LRSTR,
                              RCC->APB2RSTR, RCC->APB2RSTR,  RCC->APB4RSTR};

  // Parse through the array of possible SPI peripherals and find the one we are
  // at
  for (int i = 0; i < sizeof(SPIx_BASE_ADDRS) / sizeof(SPI_RegDef_t *); i++) {
    if (*spi_reg != SPIx_BASE_ADDRS[i])
      continue;

    // SPIx_RCC_REG[i] |= (1 << SPIx_EN_BIT_POS[i]);
    // // Reset the peripheral by turning reset bit on/off
    // delay(1);
    // SPIx_RCC_REG[i] &= ~(1 << SPIx_EN_BIT_POS[i]);
    // delay(1);
    break;
  }

  // First disable SPE so SPI can be written
  (*spi_reg)->CR1 &= ~(1 << SPI_CR1_SPE);

  uint32_t tmp_cfg_word = 0;
  tmp_cfg_word |= (cfg->bus_config) << SPI_CFG2_COMM;
  tmp_cfg_word |= (cfg->cpha) << SPI_CFG2_CPHA;
  tmp_cfg_word |= (cfg->cpol) << SPI_CFG2_CPOL;
  tmp_cfg_word |= (cfg->device_mode) << SPI_CFG2_MASTER;

  // Not sure I need the outer if statement. Only allows for SSM when in master
  // mode.
  if (cfg->device_mode == SPI_DEVICE_MODE_MASTER) {
    if (cfg->ssm == SPI_SSM_ENABLE) {
      tmp_cfg_word |= (1 << SPI_CFG2_SSM);
      tmp_cfg_word &= ~(1 << SPI_CFG2_SSOM);
    } else {
      tmp_cfg_word |= (1 << SPI_CFG2_SSOE);
      tmp_cfg_word &= ~(1 << SPI_CFG2_SSM);
    }
  }

  (*spi_reg)->CFG2 = tmp_cfg_word;
  (*spi_reg)->CFG2 = tmp_cfg_word;

  // Set the baud rate and data frame size
  (*spi_reg)->CFG1 = ((uint8_t)cfg->baud_divisor) << SPI_CFG1_MBR;
  (*spi_reg)->CFG1 |= ((uint8_t)cfg->dff) << SPI_CFG1_DSIZE;

  // Enable the SPI register
  (*spi_reg)->CR1 |= 1;
  (*spi_reg)->CR1 |= 1;
  (*spi_reg)->CR1 |= 1;
  (*spi_reg)->CR1 |= 1;
  (*spi_reg)->CR1 |= 1;
  (*spi_reg)->CR1 |= 1;

  int asdf=0;
}

void SPI_deinit(SPI_RegDef_t *p_SPI_x) {
  // First set IOLOCK bit to modify register
  p_SPI_x->CR1 &= ~((1 << SPI_CR1_IOLOCK) + (1 << SPI_CR1_SPE));

  // Reset the SPI peripheral before configuring
  static SPI_RegDef_t *const SPIx_BASE_ADDRS[6] = {SPI1, SPI2, SPI3,
                                                   SPI4, SPI5, SPI6};

  static uint8_t const SPIx_EN_BIT_POS[6] = {12, 14, 15, 13, 20, 5};

  uint32_t SPIx_RCC_REG[6] = {RCC->APB2RSTR, RCC->APB1LRSTR, RCC->APB1LRSTR,
                              RCC->APB2RSTR, RCC->APB2RSTR,  RCC->APB4RSTR};

  // Parse through the array of possible SPI peripherals and find the one we are
  // at
  for (int i = 0; i < sizeof(SPIx_BASE_ADDRS) / sizeof(SPI_RegDef_t *); i++) {
    if (p_SPI_x != SPIx_BASE_ADDRS[i])
      continue;

    // Reset the peripheral by turning reset bit on/off
    SPIx_RCC_REG[i] |= (1 << SPIx_EN_BIT_POS[i]);

    // Add a delay to make sure the reset is complete
    delay(1);

    SPIx_RCC_REG[i] &= ~(1 << SPIx_EN_BIT_POS[i]);

    break;
  }

  // Lock the SPI registers so SPI is ready for use
  p_SPI_x->CR1 |= (1 << SPI_CR1_IOLOCK);
}

/**
 * SPI function for sending data
 * */
void SPI_send(SPI_RegDef_t *p_SPI_x, uint8_t *p_tx_buffer, uint32_t len) {
  while (len > 0) {
    // Wait until TXE is set
    while (!(p_SPI_x->SR & SPI_SR_TXP))
      ;

    // From the word size, calculate the amount of bits required to get the next
    // word from the buffer
    int bit_count = ((p_SPI_x->CFG1 & (0x1F << SPI_CFG1_DSIZE)) - 1) / 8 + 1;

    // Will be used to transmit via SPI
    uint32_t tx_word = 0;

    // Convert dataframe size into the amount of bytes, and therefore the amount
    // of times len has to be decremented
    if (bit_count == 1) {
      tx_word = *((uint8_t *)p_tx_buffer);
      len--;
    } else if (bit_count == 2) {
      tx_word = *((uint16_t *)p_tx_buffer);
      len -= 2;
    } else if (bit_count == 3) {
      tx_word = *((uint32_t *)p_tx_buffer) & 0xFFFFFF;
      len -= 3;
    } else if (bit_count == 4) {
      tx_word = *((uint32_t *)p_tx_buffer);
      len -= 4;
    }

    // Send word
    p_SPI_x->TXDR = tx_word;
  }
}

/**
 * SPI function for receiving data
 **/
void SPI_receive(SPI_RegDef_t *p_SPI_x, uint8_t *p_rx_buffer, uint32_t len);
