/*
 * STM32H723xx_gpio.h
 *
 *  Created on: Oct. 1, 2023
 *      Author: jbates
 */

#ifndef INC_STM32H723XX_GPIO_H_
#define INC_STM32H723XX_GPIO_H_

#include "stm32h723xx.h"

typedef struct {
	uint8_t GPIO_pin_number;
	uint8_t GPIO_pin_mode;
	uint8_t GPIO_pin_speed;
	uint8_t GPIO_pin_pupd_control;
	uint8_t GPIO_pin_out_type;
	uint8_t GPIO_pin_alt_func_mode;
} GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *p_GPIO_x;				//Holds the base address of the GPIO port which the pin belongs
	GPIO_PinConfig_t GPIO_pin_config;	//Holds the GPIO pin configuration settings
} GPIO_Handle_t;

/*
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

/*
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_PUPDR_NONE			0
#define GPIO_PUPDR_PU			1
#define GPIO_PUPDR_PD			2


/*
 * Peripheral clock setup
 */
void GPIO_peri_clock_control(GPIO_RegDef_t *p_gpio_x, uint8_t en_state);

/*
 * Init and de-init of GPIO
 */
void GPIO_init(GPIO_Handle_t* p_gpio_handle);
void GPIO_deinit(GPIO_RegDef_t *p_gpio_x);

/*
 * Data read and write
 */
uint8_t GPIO_read_from_input_pin(GPIO_RegDef_t *p_gpio_x, uint8_t pin);
uint16_t GPIO_read_from_input_port(GPIO_RegDef_t *p_gpio_x);
void GPIO_write_to_output_pin(GPIO_RegDef_t *p_gpio_x, uint8_t pin, uint8_t val);
void GPIO_write_to_output_port(GPIO_RegDef_t *p_gpio_x, uint16_t val);
void GPIO_toggle_output_pin(GPIO_RegDef_t *p_gpio_x, uint8_t pin);

/*
 * IRQ configuration and IRQ handling
 */
void GPIO_irq_interrupt_config(uint8_t irq_number, uint8_t en_state);
void GPIO_irq_priority_config(uint8_t irq_number, uint8_t irq_priority);
int GPIO_irq_handling(uint8_t pin);

#endif /* INC_STM32H723XX_GPIO_H_ */
