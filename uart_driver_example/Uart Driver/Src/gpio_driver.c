/*
 *	gpio_driver.c - Gpio library for STM32F411xC/xE ARM microcontrollers
 *
 *	Created on: Aug 13, 2024
 *      Author: Emircan Hızarcı
 */

#include "gpio_driver.h"

static uint16_t GPIO_USED_PINS[7] = {0U};

static uint8_t GET_GPIO_Index(GPIO_TypeDef* GPIOx);

static void GPIO_ENABLE_CLOCK(GPIO_TypeDef* GPIOx);

static uint8_t IS_GPIO_PIN_USED(uint8_t GPIOx, uint16_t pin);

uint8_t GET_GPIO_Index(GPIO_TypeDef* GPIOx)
{
	return ((uint32_t)GPIOx - (GPIOA_BASE)) / ((GPIOB_BASE) - (GPIOA_BASE));
}

uint8_t IS_GPIO_PIN_USED(uint8_t GPIOx, uint16_t pin){
	return (GPIO_USED_PINS[GPIOx] & (1 << pin));
}

void GPIO_ENABLE_CLOCK(GPIO_TypeDef* GPIOx){
	RCC->AHB1ENR |=  (1 << GET_GPIO_Index(GPIOx));
}

void GPIO_init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_MODE_T OUTPUT, GPIO_OTYPE_T OTYPE, GPIO_OSPEED_T OSPEED, GPIO_PUPD_T PUPD, uint8_t alternate)
{
	if(GPIO_Pin == 0x00)
	{
		return;
	}

	uint8_t GPIO_PORT = GET_GPIO_Index(GPIOx);

	GPIO_ENABLE_CLOCK(GPIOx);

	uint8_t pin = 0;
	for (pin = 0; pin < 16; pin++){

		/// Check if pin available and if pin used///
		if (((GPIO_Pin & (1 << pin)) == 0) || (IS_GPIO_PIN_USED(GPIO_PORT, pin))) {
			continue;
		}

		GPIOx->OSPEEDR &= ~(3UL << (2 * pin));
		GPIOx->OSPEEDR |= (OSPEED << (2 * pin));


		GPIOx->OTYPER &= ~(1UL << pin);
		GPIOx->OTYPER |= (OTYPE << pin);

		if(OUTPUT != GPIO_MODE_AN)
		{
			GPIOx->PUPDR &= ~(3UL << (2 * pin));
			GPIOx->PUPDR |= (PUPD << (2 * pin));
		}

		if(OUTPUT == GPIO_MODE_AF){
			GPIOx->AFR[pin >> 3U] &= ~(15UL << (4 * (pin & 0x07)));
			GPIOx->AFR[pin >> 3U] |= (alternate << (4 * (pin & 0x07)));
		}

		GPIOx->MODER &= ~(3UL << (2 * pin));
		GPIOx->MODER |= (OUTPUT << (2 * pin));

		GPIO_USED_PINS[GET_GPIO_Index(GPIOx)] |= (1U << pin);
	}
}
