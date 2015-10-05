/*
 * Copyright (C) 2015 Rakendra Thapa <rakendrathapa@gmail.com
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_lm4f120
 * @{
 *
 * @file        timer.c
 * @brief       Implementation of the low-level timer driver for the LM4F120
 *
 * @author      Rakendra Thapa <rakendrathapa@gmail.com>
 */

#include <stdint.h>

#include "cpu.h"
#include "sched.h"
#include "thread.h"
#include "periph_conf.h"
#include "periph/timer.h"
#include "mutex.h"

//#include "timer_lm4f120.h"
#define ENABLE_DEBUG (0)
#include "debug.h"
/* guard file in case no timers are defined */
#if TIMER_0_EN

/**
 * @brief Struct holding the configuration data
 * @{
 */
typedef struct {
    void (*cb)(int);            /**< timeout callback */
    unsigned int diviser; /**< software clock diviser */
} timer_conf_t;

static timer_conf_t config[TIMER_NUMOF];
/**@}*/

#include "hw_timer.h"

// Missing from driverlib ?!
static inline unsigned long
PRIV_TimerPrescaleSnapshotGet(unsigned long ulBase, unsigned long ulTimer)
{
    return((ulTimer == TIMER_A) ? HWREG(ulBase + TIMER_O_TAPS) :
           HWREG(ulBase + TIMER_O_TBPS));
}

static inline unsigned long long _scaled_to_ll_value(unsigned int uncorrected, unsigned int diviser){
  //  unsigned long long current_clock = ROM_SysCtlClockGet() / 1000000; // in Mhz, us per clock tick
  //printf ("TIMER2C uncorrect: %x, clock : %lx\n", uncorrected, (unsigned long)current_clock);
  // This will fit in the 32 + 16 bits even at 80Mhz.
  unsigned long long scaledv = (unsigned long long) uncorrected * diviser;
  // printf ("TIMER2C corrected: %x:%x\n", (unsigned int) (scaledv>>32), (unsigned int) (scaledv & 0xFFFFFFFF));
  return scaledv;
}

static inline unsigned int _llvalue_to_scaled_value(unsigned long long corrected, unsigned int diviser){
  //  unsigned long current_clock = ROM_SysCtlClockGet() / 1000000; // in Mhz, us per clock tick
  // This will fit in the 32 + 16 bits even at 80Mhz.
  unsigned long long scaledv = corrected / diviser;
  return scaledv;
}

int timer_init(tim_t dev, unsigned int us_per_tick, void (*callback)(int))
{
    if (dev >= TIMER_NUMOF){
      return -1;
    }

    config[dev].cb = callback;                          /* User Function */
    config[dev].diviser = us_per_tick * ROM_SysCtlClockGet()/1000000;

    unsigned int sysctl_timer;
    unsigned int timer_base;
    unsigned int timer_side = TIMER_A;
    unsigned int timer_cfg = TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC_UP;
    unsigned int timer_max_val;
    unsigned int timer_intbit = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;

    switch(dev){
#if TIMER_0_EN
    case TIMER_0:
      sysctl_timer = SYSCTL_PERIPH_WTIMER0;
      timer_base = WTIMER0_BASE;
      timer_max_val = TIMER_0_MAX_VALUE;
      break;
#endif
#if TIMER_1_EN
    case TIMER_1:
      sysctl_timer = SYSCTL_PERIPH_WTIMER1;
      timer_base = WTIMER1_BASE;
      timer_max_val = TIMER_1_MAX_VALUE;
      break;
#endif
    default:
      return -1; // unreachable
    }

    if (timer_side == TIMER_A) {
      timer_cfg |= TIMER_TAMR_TAMIE;
    } else if (timer_side == TIMER_B){
      timer_cfg |= TIMER_TBMR_TBMIE;
    }

    ROM_SysCtlPeripheralEnable(sysctl_timer);
    /* int i = 0xFFFF; */
    /* while((volatile int)i--); */

    ROM_TimerDisable(timer_base, timer_side);

    //        WTIMER0_CTL_R &= ~0x00000001;                       /* Disable timer0A during setup */

    ROM_TimerConfigure(timer_base, timer_cfg);

    //	ROM_TimerPrescaleSet(WTIMER0_BASE, TIMER_A, 0xFFFF - prescaler);
    unsigned long long lltimer_val_max = _scaled_to_ll_value(timer_max_val, config[dev].diviser);

    ROM_TimerPrescaleSet(timer_base, timer_side, lltimer_val_max >> 32);
        /* WTIMER0_CFG_R  = TIMER_CFG_16_BIT; */
        /* WTIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;            /\* Configure for periodic mode *\/ */
	/* ROM_TimerPrescaleSet(WTIMER0_BASE, TIMER_A, prescaler); */
    ROM_TimerLoadSet(timer_base, timer_side, lltimer_val_max & 0xFFFFFFFF);
        //WTIMER0_TAPR_R = TIMER_0_PRESCALER;                 /* 1us timer0A */
    ROM_TimerIntClear(timer_base, timer_intbit);
	
        //WTIMER0_ICR_R  = 0x00000001;                        /* clear timer0A timeout flag */
        //WTIMER0_IMR_R |= 0x00000001;                        /* arm timeout interrupt */

    ROM_TimerIntEnable(timer_base, timer_intbit);

    timer_irq_enable(dev);
    timer_start(dev);

    /* DEBUG("startTimeout Value=0x%lx prescale 0x%lx/%lu\n", */
    /* 	  ROM_TimerLoadGet(WTIMER0_BASE, TIMER_A), */
    /* 	  ROM_TimerPrescaleGet(WTIMER0_BASE, TIMER_A), */
    /* 	  ROM_TimerPrescaleGet(WTIMER0_BASE, TIMER_A)); */
    return 1;
    /* } */
    /* return -1; */
}

int timer_set(tim_t dev, int channel, unsigned int timeout)
{
    if (dev >= TIMER_NUMOF){
      return -1;
    }

    switch(dev){
#if TIMER_0_EN
    case TIMER_0:
      break;
#endif
#if TIMER_1_EN
    case TIMER_1:
      break;
#endif
    default:
      return -1; // unreachable
    }

    /* if (dev == TIMER_0) { */
    unsigned int correct_now = timer_read(dev);
    unsigned long long uncorrected_now = _scaled_to_ll_value(correct_now, config[dev].diviser);
    /* printf("timer_set now(corrected)=0x%x (uncorrect)=0x%llx\n", correct_now, uncorrected_now); */
    /* printf("timer_set timeout=0x%x\n", timeout); */
    int  r = timer_set_absolute(dev, channel, uncorrected_now+timeout);
    /* unsigned int correct_after = timer_read(dev); */
    /* printf("start %u end %u, diff %u\n", correct_now, correct_after ,correct_after - correct_now); */

    return r;
}

int timer_set_absolute(tim_t dev, int channel, unsigned int value)
{
  unsigned int timer_base;
  unsigned int timer_side = TIMER_A;
  //unsigned int timer_intbit = TIMER_CAPA_EVENT;
  /* unsigned int timer_intbit = TIMER_TIMA_TIMEOUT; */

  if (dev >= TIMER_NUMOF){
    return -1;
  }

  switch(dev){
#if TIMER_0_EN
  case TIMER_0:
    timer_base = WTIMER0_BASE;
    break;
#endif
#if TIMER_1_EN
  case TIMER_1:
    timer_base = WTIMER1_BASE;
    break;
#endif
  default:
    return -1; // unreachable
    break;
  }
  ROM_TimerDisable(timer_base, timer_side);
  
  /* unsigned int correct_now = timer_read(dev);  */
  /* if (dev == TIMER_0) { */
  /* // This will fit in the 32 + 16 bits even at 80Mhz */
  /* unsigned long long scaledv = value * current_clock; */
  unsigned long long scaledv = _scaled_to_ll_value(value, config[dev].diviser);
  //ROM_TimerIntEnable(timer_base, timer_intbit);

  /* if (scaledv>>32){ */
  /*   ROM_TimerPrescaleSet(timer_base, timer_side, scaledv >> 32); */
  /* } else { */
  /*   ROM_TimerPrescaleSet(timer_base, timer_side, 0); */
  /* } */
  if (scaledv>>32){
    ROM_TimerPrescaleMatchSet(timer_base, timer_side, scaledv >> 32);
  } else {
    ROM_TimerPrescaleMatchSet(timer_base, timer_side, 0);
  }

  /* ROM_TimerLoadSet(timer_base, timer_side, (unsigned long) (scaledv & 0xFFFFFFFF)); */
  ROM_TimerMatchSet(timer_base, timer_side, (unsigned long) (scaledv & 0xFFFFFFFF));
  //WTIMER0_TAILR_R = 0x00000000 | value;               /* period; Reload value */
  /* printf("Setting timer absolute value=0x%x / 0x%x:%x\n", */
  /* 	     value, */
  /* 	     (unsigned int) (scaledv>>32), */
  /* 	     (unsigned int)scaledv & 0xFFFFFFFF); */
  /* unsigned long long now = _scaled_to_ll_value(timer_read(dev), config[dev].diviser); */
  /* printf("Current time %x:%x\n", (unsigned int)(now>>32), (unsigned int)now & 0xFFFFFFFF); */
  /* printf("Setting timer match value=0x%x:0x%x / 0x%x:0x%x\n", */
  /* 	 (unsigned int)((scaledv)>>32), */
  /* 	 (unsigned int)scaledv & 0xFFFFFFFF, */
  /* 	 (unsigned int)ROM_TimerPrescaleMatchGet(timer_base, timer_side), */
  /* 	 (unsigned int)ROM_TimerMatchGet(timer_base,timer_side)); */

  ROM_TimerEnable(timer_base, timer_side);

  /* printf("Current timer value=0x%lx\n", ROM_TimerValueGet(WTIMER0_BASE, TIMER_A)); */
  return 1;
  /* } */
  /* unsigned int correct_after  =        timer_read(dev); */
  /* printf("start %u end %u, diff %u\n", correct_now, correct_after ,correct_after - correct_now); */

  /* return -1; */
}

int timer_clear(tim_t dev, int channel)
{
    unsigned int timer_intbit = TIMER_TIMA_TIMEOUT;
    unsigned int timer_base;

    if (dev >= TIMER_NUMOF){
      return -1;
    }

    switch(dev){
#if TIMER_0_EN
    case TIMER_0:
      timer_base = WTIMER0_BASE;
      break;
#endif
#if TIMER_1_EN
    case TIMER_1:
      timer_base = WTIMER1_BASE;
      break;
#endif
    default:
      return -1; // unreachable
      break;
    }


    /* if (dev == TIMER_0){ */
    ROM_TimerIntClear(timer_base, timer_intbit);
      //        WTIMER0_ICR_R = TIMER_ICR_TATOCINT;
    return 1;
    /* } */
    /* return -1; */
}

unsigned int timer_read(tim_t dev)
{
  /* unsigned int now = HWREG(WTIMER0_BASE + TIMER_O_TAR); */
  unsigned int timer_base;
  unsigned int timer_side = TIMER_A;

  if (dev >= TIMER_NUMOF){
    return -1;
  }

  switch(dev){
#if TIMER_0_EN
  case TIMER_0:
    timer_base = WTIMER0_BASE;
    break;
#endif
#if TIMER_1_EN
  case TIMER_1:
    timer_base = WTIMER1_BASE;
    break;
#endif
  default:
    return -1; // unreachable
    break;
  }

  /* if (dev == TIMER_0) { */
  /* unsigned int currTimer0Val=0; */
  /* unsigned int loadTimer0Val=0; */
  /* currTimer0Val = (unsigned int)ROM_TimerValueGet(WTIMER0_BASE, TIMER_A); */
  /* loadTimer0Val = (unsigned int)ROM_TimerLoadGet(WTIMER0_BASE, TIMER_A); */
  /* DEBUG("WTIMER0_TAILR_R=0x%lx\t currTimer0Val=0x%x\t loadTimer0Val=0x%x\n", WTIMER0_TAILR_R, currTimer0Val, loadTimer0Val); */
  /* return (loadTimer0Val - currTimer0Val); */
  unsigned long long high_bits = ((unsigned long long)PRIV_TimerPrescaleSnapshotGet(timer_base, timer_side)) << 32;
  unsigned long long low_bits = (unsigned long long)ROM_TimerValueGet(timer_base, timer_side);

  /* printf("TREAD prescale %lx\n", (unsigned long) (high_bits>>32)); */
  /* printf("TREAD timer %lx\n", (unsigned long) low_bits); */
      
  unsigned long long total = high_bits + low_bits;
  DEBUG("Combined %lx:%lx\n", (unsigned long) (total>>32), (unsigned long) (total & 0xFFFFFFFF));

  unsigned int scaled_value = _llvalue_to_scaled_value(total, config[dev].diviser);

  /* unsigned long current_clock = ROM_SysCtlClockGet() / 1000000; // in Mhz, us per clock tick */
  /* unsigned int scaled_value = total/current_clock; */
  /* printf("Scaled value: %x (timer value: %lx:%lx)\n", */
  /* 	    scaled_value, */
  /* 	    (unsigned long) (total>>32), */
  /* 	    (unsigned long) (total & 0xFFFFFFFF)); */

  /* unsigned int now2 = HWREG(WTIMER0_BASE + TIMER_O_TAR); */

  /* printf ("test : %x / %x\n", */
  /* 	      _llvalue_to_scaled_value(_scaled_to_ll_value(0xFFFFFFFF)), */
  /* 	      now2-now); */

  return scaled_value;
  /* } */
  /* return 0; */
}

void timer_start(tim_t dev)
{
    unsigned int timer_base;
    unsigned int timer_side = TIMER_A;

    if (dev >= TIMER_NUMOF){
      return ;
    }

    switch(dev){
#if TIMER_0_EN
    case TIMER_0:
      timer_base = WTIMER0_BASE;
      break;
#endif
#if TIMER_1_EN
    case TIMER_1:
      timer_base = WTIMER1_BASE;
      break;
#endif
    default:
      return; // unreachable
    }

    ROM_TimerEnable(timer_base, timer_side);
}

void timer_stop(tim_t dev)
{
    unsigned int timer_base;
    unsigned int timer_side = TIMER_A;

    if (dev >= TIMER_NUMOF){
      return;
    }

    switch(dev){
#if TIMER_0_EN
    case TIMER_0:
      timer_base = WTIMER0_BASE;
      break;
#endif
#if TIMER_1_EN
    case TIMER_1:
      timer_base = WTIMER1_BASE;
      break;
#endif
    default:
      return; // unreachable
    }

    ROM_TimerDisable(timer_base, timer_side);
}

void timer_irq_enable(tim_t dev)
{
  if (dev >= TIMER_NUMOF){
    return;
  }

  unsigned int timer_intbase;

  switch(dev){
#if TIMER_0_EN
  case TIMER_0:
    timer_intbase = INT_WTIMER0A;
    break;
#endif
#if TIMER_1_EN
  case TIMER_1:
    timer_intbase = INT_WTIMER1A;
    break;
#endif
  default:
    return; // unreachable
  }

  ROM_IntPrioritySet(timer_intbase, 32);
  ROM_IntEnable(timer_intbase);
}

void timer_irq_disable(tim_t dev)
{
  if (dev >= TIMER_NUMOF){
    return;
  }
  unsigned int timer_base;
  unsigned int timer_intbit = TIMER_TIMA_TIMEOUT;
  unsigned int timer_intbase;

  switch(dev){
#if TIMER_0_EN
  case TIMER_0:
    timer_base = WTIMER0_BASE;
    timer_intbase = INT_WTIMER0A;
    break;
#endif
#if TIMER_1_EN
  case TIMER_1:
    timer_base = WTIMER1_BASE;
    timer_intbase = INT_WTIMER1A;
    break;
#endif
  default:
    return; // unreachable
  }

  ROM_IntEnable(timer_intbase);
  ROM_TimerIntDisable(timer_base, timer_intbit);
}

void timer_reset(tim_t dev)
{
    if (dev >= TIMER_NUMOF){
      return;
    }

    unsigned int sysctl_timer;

    switch(dev){
#if TIMER_0_EN
    case TIMER_0:
      sysctl_timer = SYSCTL_PERIPH_WTIMER0;
      break;
#endif
#if TIMER_1_EN
    case TIMER_1:
      sysctl_timer = SYSCTL_PERIPH_WTIMER1;
      break;
#endif
    default:
      return; // unreachable
    }

    /* Performs a software reset of a peripheral */
    ROM_SysCtlPeripheralReset(sysctl_timer);
}

#if TIMER_0_EN
/* void isr_timer0a(void) */
/* { */
/*   ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); */
/*       //    TIMER0_ICR_R = TIMER_ICR_TATOCINT; */
/*     config[TIMER_0].cb(0); */

/*     if (sched_context_switch_request){ */
/*         thread_yield(); */
/*     } */
/* } */
void isr_wtimer0a(void)
{
  // Clears both IT
  unsigned long status = ROM_TimerIntStatus(WTIMER0_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
  printf("Timeout : %lx, Match : %lx\n", status & TIMER_TIMA_TIMEOUT, status & TIMER_TIMA_MATCH);

  ROM_TimerIntClear(WTIMER0_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
  //   WTIMER0_ICR_R = TIMER_ICR_TATOCINT;

    config[TIMER_0].cb(0);
    if (sched_context_switch_request){
        thread_yield();
    }
}
#endif /* TIMER_0_EN */

#if TIMER_1_EN
void isr_wtimer1a(void)
{
    ROM_TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
  //   WTIMER0_ICR_R = TIMER_ICR_TATOCINT;

    config[TIMER_1].cb(0);
    if (sched_context_switch_request){
        thread_yield();
    }
}
#endif /* TIMER_1_EN */

#endif /* TIMER_0_EN */
/** @} */
