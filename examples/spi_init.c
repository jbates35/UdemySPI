
#define FAST 100000
#define MEDIUM 300000
#define SLOW 1000000
#define WAIT(CNT)                                                              \
  do {                                                                         \
    for (int def_i = 0; def_i < CNT; def_i++)                                  \
      ;                                                                        \
  } while (0)

/******* PINS *********/
#define LED_GREEN_PORT GPIOB
#define LED_GREEN_PIN 0
#define LED_YELLOW_PORT GPIOE
#define LED_YELLOW_PIN 1
#define LED_RED_PORT GPIOB
#define LED_RED_PIN 14

#define USER_PBUTTON_PORT GPIOC
#define USER_PBUTTON_PIN 13

#define SPI1_GPIO_PORT GPIOA
#define SPI1_CLOCK_PIN 5
#define SPI1_NSS_PIN 4
#define SPI1_MISO_PIN 6
#define SPI1_MOSI_PIN 7

#define SPI4_GPIO_PORT GPIOE
#define SPI4_CLOCK_PIN 2
#define SPI4_NSS_PIN 4
#define SPI4_MISO_PIN 5
#define SPI4_MOSI_PIN 6

#include <stdint.h>
#include <string.h>

#include "stm32h723xx.h"
#include "stm32h723xx_gpio.h"
#include "stm32h723xx_spi.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning                                                                       \
    "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* Taken from the reference manual */
void start_SPI_manually() {

  SYSCFG_PCLK_EN();

  /* 1. */
  ////////////// Set up the proper GPIO registers /////////////////
  GPIO_peri_clock_control(GPIOA, ENABLE);
  GPIO_peri_clock_control(GPIOB, ENABLE);
  GPIO_peri_clock_control(GPIOC, ENABLE);
  GPIO_peri_clock_control(GPIOD, ENABLE);
  GPIO_peri_clock_control(GPIOE, ENABLE);
  GPIO_peri_clock_control(GPIOF, ENABLE);
  GPIO_peri_clock_control(GPIOG, ENABLE);

  // Create GPIO handler and assign shortcuts for easier access
  GPIO_Handle_t gpio_handle;
  GPIO_RegDef_t **addr = &gpio_handle.p_GPIO_x;
  GPIO_PinConfig_t *cfg = &gpio_handle.GPIO_pin_config;

  // SPI 4 Init
  *addr = SPI1_GPIO_PORT;
  cfg->GPIO_pin_mode = GPIO_MODE_ALTFN;
  cfg->GPIO_pin_speed = GPIO_SPEED_HIGH;
  cfg->GPIO_pin_pupd_control = GPIO_PUPDR_NONE;
  cfg->GPIO_pin_out_type = GPIO_OP_TYPE_PUSHPULL;
  cfg->GPIO_pin_alt_func_mode = 5;

  // PE4 - SPI_4_NSS
  cfg->GPIO_pin_number = SPI1_NSS_PIN;
  GPIO_init(&gpio_handle);

  // PE5 - SPI_4_MISO
  cfg->GPIO_pin_number = SPI1_MISO_PIN;
  GPIO_init(&gpio_handle);

  // PE6 - SPI_4_MOSI
  cfg->GPIO_pin_number = SPI1_MOSI_PIN;
  GPIO_init(&gpio_handle);

  // PE2 - SPI_4_SCK
  cfg->GPIO_pin_number = SPI1_CLOCK_PIN;
  GPIO_init(&gpio_handle);

  SPI_peri_clock_control(SPI1, ENABLE);
  SPI_peri_clock_control(SPI2, ENABLE);
  SPI_peri_clock_control(SPI3, ENABLE);
  SPI_peri_clock_control(SPI4, ENABLE);
  SPI_peri_clock_control(SPI5, ENABLE);
  SPI_peri_clock_control(SPI6, ENABLE);

  /* 2. */
  /* Write to the SPI_CFG1 and SPI_CFG2 registers to set up proper values of all
   * not reserved bits and bit fields included there with next exceptions:*/

  /* a)SSOM, SSOE, MBR[2:0], MIDI[3:0] and MSSI[3:0] are required and taken into
   * account at master mode exclusively. */
  /* CFG1 */
  uint32_t tmp_mbr = 0b100 << 28;
  uint32_t tmp_cfg1 = tmp_mbr;
  SPI1->CFG1 = tmp_cfg1;

  /* CFG2 */
  uint32_t tmp_ssoe = 0b1 << 29;
  uint32_t tmp_ssm = 0b1 << 28;
  uint32_t tmp_master = 0b1 << 22;

  uint32_t tmp_cfg2 = tmp_ssoe | tmp_ssm | tmp_master;
  SPI1->CFG2 = tmp_cfg2;

  /* b)UDRDET[1:0] and UDRCFG[1:0] are required and taken into account at slave
   * mode only. The MBR[2:0] setting is taken into account only when slave is
   * configured at TI mode. */

  /* c)CRCSIZE[4:0] is required if CRCEN is set, */

  /* d)CPOL, CPHA, LSBFRST, SSOM, SSOE, SSIOP, MSSI, MIDI and SSM are not
   * required at TI mode. */

  /* e)Once the AFCNTR bit is set at SPI_CFG2 register, all the SPI outputs
   * start to be propagated onto the associated GPIO pins regardless the
   * peripheral enable so any later configurations changes of the SPI_CFG1 and
   * SPI_CFG2 registers can affect level of signals at these pins. */

  /* f)The I2SMOD bit at SPI_I2SCFGR register has to be kept cleared to prevent
   * any unexpected influence of occasional I2S configuration. */

  /* 3.Write to the SPI_CR2 register to select length of the transfer, if it is
   * not known TSIZE has to be programmed to zero. */

  /* 4.Write to SPI_CRCPOLY and into TCRCINI, RCRCINI and CRC33_17 bits at
   * SPI_CR1 register to configure the CRC polynomial and CRC calculation if
   * needed. */

  /* 5.Configure DMA streams dedicated for the SPI Tx and Rx in DMA registers if
   * the DMA streams are used (see chapter Communication using DMA). */

  /* 6.Configure SSI, HDDIR and MASRX at SPI_CR1 register if required. */

  /* 7.Program the IOLOCK bit in the SPI_CFG1 register if the configuration
   * protection is required (for safety). */
  SPI1->CR1 = 0b1;

  int asdfasdfasdf = 0;
}
