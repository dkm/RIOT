/*
 * Copyright (C) 2015 Freie UniversitÃ¤t Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup         cpu_cc430
 * @{
 *
 * @file
 * @brief           CPU specific definitions for internal peripheral handling
 *
 * @author          Hauke Petersen <hauke.peterse@fu-berlin.de>
 * @author          Marc Poulhiès <dkm@kataplop.net>
 */

#ifndef CPU_PERIPH_H_
#define CPU_PERIPH_H_

#include "cpu.h"
#include "cc430_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Define a custom type for GPIO pins
 * @{
 */
#define HAVE_GPIO_T
typedef uint16_t gpio_t;
/** @} */

/**
 * @brief   Definition of a fitting UNDEF value
 */
#define GPIO_UNDEF          (0xffff)

/**
 * @brief   Mandatory function for defining a GPIO pins
 * @{
 */
#define GPIO_PIN(x, y)      ((gpio_t)(((x & 0xff) << 8) | (1 << (y & 0xff))))
/** @} */

/**
 * @brief   Override direction values
 * @{
 */
#define HAVE_GPIO_DIR_T
typedef enum {
    GPIO_DIR_IN =  0x00,    /**< configure pin as input */
    GPIO_DIR_OUT = 0xff,    /**< configure pin as output */
} gpio_dir_t;
/** @} */
  
#ifdef __cplusplus
}
#endif

#endif /* CPU_PERIPH_H_ */
/** @} */
