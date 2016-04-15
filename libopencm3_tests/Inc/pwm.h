#ifndef PWM_H
#define PWM_H

#include <libopencm3/stm32/timer.h>
#include <stdint.h>

typedef struct {
  //uint32_t timer; hardcoded to TIM4
  //uint32_t channels_used; all 4 channels
  unsigned int frequency;
  unsigned int resolution;
}Pwm4;

void pwm4_init(Pwm4* pwm4);
void pwm4_set(Pwm4* pwm4, enum tim_oc_id oc_id, uint32_t value);

void pwm4_disable_invert(Pwm4* pwm4, enum tim_oc_id oc_id);
void pwm4_disable_invert(Pwm4* pwm4, enum tim_oc_id oc_id);

#endif
