/*
 *	gpio_driver.h - Gpio library for STM32F411xC/xE ARM microcontrollers
 *
 *	Created on: Aug 13, 2024
 *      Author: Emircan Hızarcı
 */

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

/* C++ */
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

#ifndef	GPIO_PIN_0
#define GPIO_PIN_0                 ((uint16_t)0x0001)
#define GPIO_PIN_1                 ((uint16_t)0x0002)
#define GPIO_PIN_2                 ((uint16_t)0x0004)
#define GPIO_PIN_3                 ((uint16_t)0x0008)
#define GPIO_PIN_4                 ((uint16_t)0x0010)
#define GPIO_PIN_5                 ((uint16_t)0x0020)
#define GPIO_PIN_6                 ((uint16_t)0x0040)
#define GPIO_PIN_7                 ((uint16_t)0x0080)
#define GPIO_PIN_8                 ((uint16_t)0x0100)
#define GPIO_PIN_9                 ((uint16_t)0x0200)
#define GPIO_PIN_10                ((uint16_t)0x0400)
#define GPIO_PIN_11                ((uint16_t)0x0800)
#define GPIO_PIN_12                ((uint16_t)0x1000)
#define GPIO_PIN_13                ((uint16_t)0x2000)
#define GPIO_PIN_14                ((uint16_t)0x4000)
#define GPIO_PIN_15                ((uint16_t)0x8000)
#define GPIO_PIN_All               ((uint16_t)0xFFFF)
#define GPIO_PIN_MASK
#endif

typedef enum{
	GPIO_MODE_IN = 0x00,
	GPIO_MODE_OUT,
	GPIO_MODE_AF,
	GPIO_MODE_AN
}GPIO_MODE_T;

typedef enum{
	GPIO_OTYPE_PP = 0x00,
	GPIO_OTYPE_OD,
}GPIO_OTYPE_T;

typedef enum{
	GPIO_OSPEED_LOW = 0x00,
	GPIO_OSPEED_MEDIUM,
	GPIO_OSPEED_FAST,
	GPIO_OSPEED_HIGH
}GPIO_OSPEED_T;

typedef enum{
	GPIO_PUPD_NOPULL = 0x00,
	GPIO_PUPD_PULL_UP,
	GPIO_PUPD_PULL_DOWN,
	GPIO_PUPD_RESERVED
}GPIO_PUPD_T;



void GPIO_init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_MODE_T OUTPUT, GPIO_OTYPE_T OTYPE, GPIO_OSPEED_T OSPEED, GPIO_PUPD_T PUPD, uint8_t alternate);

/* C++ */
#ifdef __cplusplus
}
#endif

#endif /* GPIO_DRIVER_H_ */
