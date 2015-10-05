/*
 * Copyright (C) 2015 Marc Poulhiès
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_lm4f120
 * @{
 *
 * @file
 * @brief       Low-level PWM driver implementation
 *
 * @author      Marc Poulhiès <dkm@kataplop.net>
 *
 * @}
 */

#include <stdint.h>
#include <string.h>

#include "log.h"
#include "cpu.h"
#include "board.h"
#include "periph/pwm.h"
#include "periph/gpio.h"
#include "periph_conf.h"


#define ENABLE_DEBUG (1)
#include "debug.h"

/* ignore file in case no PWM devices are defined */
#if PWM_NUMOF

typedef struct pwm_conf_s {
  uint8_t pin_num;
  uint32_t ccp;
  uint32_t timer;
} pwm_conf_t;

#define PWM_0_GPIO_PIN GPIO_PIN_4
#define PWM_0_GPIO_SYSCTL_PORT SYSCTL_PERIPH_GPIOB
#define PWM_0_GPIO_PORT_BASE GPIO_PORTB_BASE
#define PWM_0_CCP GPIO_PB4_T1CCP0
#define PWM_0_TIMER TIMER1_BASE
#define PWM_0_TIMER_SIDE TIMER_A
#define PWM_0_TIMER_SYSCTL SYSCTL_PERIPH_TIMER1
#define PWM_0_TIMER_PWMCFG TIMER_CFG_A_PWM
  
/*
 * 4 devices, 2 channels per device.
 */

/*
 * PWM dev 0 : T0
 *         - PB3 CCP0
 *         - PB2 CCP1
 *     dev 1 : T1
 *         - PB4 CCP0
 *         - PB5 CCP1
 *     dev 2 : T2
 *          - PB0 CCP0 
 *          - PB1 CCP1
 *     dev 3 : T3
 *          - PB2 CCP0
 *          - PB3 CCP1
 */

int pwm_init(pwm_t dev, pwm_mode_t mode,
	     unsigned int frequency, unsigned int resolution)
{
    if (dev >= PWM_NUMOF) {
        return -1;
    }

    unsigned long timer_base;
    unsigned long gpio_pin;
    unsigned long ccp;
    unsigned long timer_ab;
    unsigned long gpio_sysctl;
    unsigned long gpio_port_base;
    unsigned long timer_pwm_cfg;
    
    switch (dev){
    case PWM_0:
      timer_pwm_cfg = PWM_0_TIMER_PWMCFG;
      timer_base = PWM_0_TIMER;
      gpio_pin = PWM_0_GPIO_PIN;
      ccp = PWM_0_CCP;
      timer_ab = PWM_0_TIMER_SIDE;
      gpio_sysctl = PWM_0_GPIO_SYSCTL_PORT;
      gpio_port_base = PWM_0_GPIO_PORT_BASE;
      break;

#if 0
    case PWM_1:
    case PWM_2:
#endif
    default:
      return -1;
    }

    // Enable GPIO port
    ROM_SysCtlPeripheralEnable(gpio_sysctl);
   
    ROM_GPIOPinConfigure(ccp);
    ROM_GPIOPinTypeTimer(gpio_port_base, gpio_pin);

    pwm_poweron(dev);
    ROM_TimerDisable(timer_base, timer_ab);
    ROM_TimerConfigure(timer_base, TIMER_CFG_SPLIT_PAIR | timer_pwm_cfg);

    unsigned long clock = ROM_SysCtlClockGet();

    // interface specifies we must keep the resolution
    unsigned long ticks = resolution;
    unsigned long real_freq = clock/resolution;

    //    unsigned long prescale = clock / (frequency * resolution);
    unsigned long prescale_count_extend = ticks >> 16;
    unsigned long ticks_low = ticks & 0xFFFF;

    if (prescale_count_extend & ~0xFF) {
      DEBUG("Resolution too high\n");
      return -1;
    }

    /* if (prescale & (~0xFF)){ */
    /*   DEBUG("Can't find settings for freq %u at resolution %u\n", frequency, resolution); */
    /*   return -1; */
    /* } */
    DEBUG("Prescaler set at %lu\n", prescale_count_extend);
    ROM_TimerPrescaleSet(timer_base, timer_ab, prescale_count_extend);

    //    uint32_t ticks = resolution; //clock / frequency;
    DEBUG("Setting ticks at %lu (reqf: %u, freq cpu: %lu)\n", ticks, frequency, clock);
    DEBUG("Real freq: %lu\n", real_freq);
    
    ROM_TimerMatchSet(timer_base, timer_ab, /* dutyCycle */ prescale_count_extend);
    ROM_TimerPrescaleMatchSet(timer_base, timer_ab, 0);
    ROM_TimerLoadSet(timer_base, timer_ab, /* ticks */ ticks_low);

    /* set PWM mode */
    /* switch (mode) { */
    /*     case PWM_LEFT: */
    /*     case PWM_RIGHT: */
    /*     case PWM_CENTER: */
    /*     default: */
    /*         return -1; */
    /* } */

    pwm_start(dev);

    return real_freq;
}

int pwm_set(pwm_t dev, int channel, unsigned int value)
{
    if (channel >= PWM_MAX_CHANNELS) {
        return -1;
    }

    unsigned long timer_base;
    unsigned long timer_ab;
    
    switch (dev){
    case PWM_0:
      timer_base = PWM_0_TIMER;
      timer_ab = PWM_0_TIMER_SIDE;
      break;
    default:
      return -1;
    }

    ROM_TimerMatchSet(timer_base, timer_ab, value);

    return 0;
}

void pwm_start(pwm_t dev)
{
    unsigned long timer_base = -1;
    unsigned long timer_ab;
    switch (dev){
    case PWM_0:
      timer_base = PWM_0_TIMER;
      timer_ab = PWM_0_TIMER_SIDE;
      break;
    default:
      return;
    }
    ROM_TimerEnable(timer_base, timer_ab);
}

void pwm_stop(pwm_t dev)
{
    unsigned long timer_base = -1;
    unsigned long timer_ab;
    switch (dev){
    case PWM_0:
      timer_base = PWM_0_TIMER;
      timer_ab = PWM_0_TIMER_SIDE;
      break;
    default:
      return;
    }

    ROM_TimerDisable(timer_base, timer_ab);
}

void pwm_poweron(pwm_t dev)
{
    unsigned long timer_sysctl;

    switch (dev){
    case PWM_0:
      timer_sysctl = PWM_0_TIMER_SYSCTL;
      break;
    default:
      return;
    }
    ROM_SysCtlPeripheralEnable(timer_sysctl);
}

void pwm_poweroff(pwm_t dev)
{
    unsigned long timer_sysctl;

    switch (dev){
    case PWM_0:
      timer_sysctl = PWM_0_TIMER_SYSCTL;
      break;
    default:
      return;
    }
    ROM_SysCtlPeripheralDisable(timer_sysctl);
}

#endif /* PWM_NUMOF */
