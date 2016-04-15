/*
 *
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include "libopencm3/stm32/gpio.h"
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencmsis/core_cm3.h>

#include "pwm.h"



#define RADD_GPIO GPIOA
#define RADD_PINA GPIO0
#define RADD_PINB GPIO1
#define RADD_PINC GPIO2

static void radd_init(void) {
  /// init gpios
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set(RADD_GPIO, RADD_PINA | RADD_PINB | RADD_PINC);
  gpio_mode_setup(RADD_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  RADD_PINA | RADD_PINB | RADD_PINC);
  gpio_set_output_options(RADD_GPIO, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                          RADD_PINA | RADD_PINB | RADD_PINC);


  /// init timer for waveform generation
  rcc_periph_clock_enable(RCC_TIM2);
  timer_reset(TIM2);
  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM2,  120000 / 2 / (10));
  timer_set_period(TIM2, 1000);
  timer_enable_counter(TIM2);
  nvic_enable_irq(NVIC_TIM2_IRQ);
  timer_enable_irq(TIM2, TIM_DIER_UIE);

  /// init timer for speed control
  rcc_periph_clock_enable(RCC_TIM3);
  timer_reset(TIM3);
  timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM3,  120000 / 2 / (50));
  timer_set_period(TIM3, 1000);
  timer_enable_counter(TIM3);
  nvic_enable_irq(NVIC_TIM3_IRQ);
  timer_enable_irq(TIM3, TIM_DIER_UIE);

  /// red led
  rcc_periph_clock_enable(RCC_GPIOD);
  gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
}

void radd_set_freq(int freq) {
  timer_set_prescaler(TIM2,  120000 / 2 / freq);
  timer_set_period(TIM2, 1000);
}

void tim2_isr(void) {
  if (timer_get_flag(TIM2, TIM_DIER_UIE))
    timer_clear_flag(TIM2, TIM_DIER_UIE);

  static int cnt = 0;



  switch(cnt++) {
  case 0:
    gpio_set(RADD_GPIO, RADD_PINA);
    gpio_clear(RADD_GPIO, RADD_PINB | RADD_PINC);
    break;

  case 1:
    gpio_set(RADD_GPIO, RADD_PINB);
    gpio_clear(RADD_GPIO, RADD_PINA | RADD_PINC);
    break;

  case 2:
    gpio_set(RADD_GPIO, RADD_PINC);
    gpio_clear(RADD_GPIO, RADD_PINA | RADD_PINB);
    cnt = 0;
    break;

  default:
    break;
  }

}

void tim3_isr(void) {
  if (timer_get_flag(TIM3, TIM_DIER_UIE))
    timer_clear_flag(TIM3, TIM_DIER_UIE);

  gpio_toggle(GPIOD, GPIO14);

  static uint32_t freq = 0 *(1<<16);
  uint32_t freq_target = 600 *(1<<16);

  freq += (freq - freq_target) >> 16;
  //if ( freq > freq_max ) freq = freq_max;

  radd_set_freq(freq >> 16);
}

const struct rcc_clock_scale rcc_hse_8mhz_3v3_perf =
  { /* 165MHz : sysclk = 8/pllm*plln/pllp*/
    .pllm = 8,
    .plln = 330,
    .pllp = 2,
    .pllq = 5,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE_DIV_4,
    .ppre2 = RCC_CFGR_PPRE_DIV_2,
    //.power_save = 1,
    .flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE |
    FLASH_ACR_LATENCY_7WS,
    .apb1_frequency = 41250000, /* sysclk / ppre1 */
    .apb2_frequency = 82500000, /* sysclk / ppre2 */
  };

static void gpio_setup(void)
{
  /* Enable GPIOC clock. */
  rcc_periph_clock_enable(RCC_GPIOD);

  /* Set GPIO12 (in GPIO port C) to 'output push-pull'. */
  gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE, GPIO14 | GPIO15 | GPIO12 );


  gpio_set(GPIOD, GPIO14);
  gpio_clear(GPIOD, GPIO15 | GPIO14 );
}

typedef struct {
  int numTurns;
  int numIncrements;
}EncoderPosition;

static EncoderPosition encoderPos;

void counter_enc_update(void);

void counter_enc_update(void) {
  encoderPos.numIncrements = timer_get_counter(TIM2);
}



#define LED_GREEN_PIN GPIO12
#define LED_ORANGE_PIN GPIO13
#define LED_RED_PIN GPIO14
#define LED_BLUE_PIN GPIO15
void board_leds_init() {
  rcc_periph_clock_enable(RCC_GPIOD);
  gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,
		  LED_BLUE_PIN | LED_GREEN_PIN | LED_ORANGE_PIN | LED_RED_PIN );
}



int main(void)
{
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3_perf);

  /* init leds */
  //board_leds_init();

  /* init complementary pwm */
  /* gpios */
  /* rcc_periph_clock_enable(RCC_GPIOE); /\* init gpio clk *\/ */
  /* gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9); /\* alternate func mode *\/ */
  /* gpio_set_output_options(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO9); /\* open drain *\/ */
  /* gpio_set_af(GPIOE, GPIO_AF1, GPIO9); /\* alternate func 1 -> TIM1 output *\/ */
  /* /\* timer *\/ */
  /* rcc_periph_clock_enable(RCC_TIM1); */
  /* timer_reset(RCC_TIM1); */
  /* timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP); */
  /* timer_set_prescaler(TIM1, rcc_apb1_frequency / 2 / ( 2*15000*1024 )); */
  /* timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1); */
  /* timer_enable_oc_output(TIM1, TIM_OC1); */
  /* /\* timer_enable_oc_output(TIM1, TIM_OC1N); *\/ */
  /* timer_set_oc_value(TIM1, TIM_OC1, 512); */
  /* timer_set_period(TIM1, 1024); */
  /* timer_enable_counter(TIM1); */

  /* gpios */
  rcc_periph_clock_enable(RCC_GPIOD);
  gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE,
		  GPIO12 | GPIO13 | GPIO14 | GPIO15);
  gpio_set_af(GPIOD, GPIO_AF2,
	      GPIO12 | GPIO13 | GPIO14 | GPIO15);
  /* gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, */
  /* 			  GPIO12); */
  /* timer */
  rcc_periph_clock_enable(RCC_TIM4);
  timer_reset(TIM4);
  timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_2, TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM4,  rcc_apb2_frequency  / 2 / (2* 15000 * 1024));
  timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM4, TIM_OC3, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM1);
  timer_enable_oc_output(TIM4, TIM_OC1);
  timer_enable_oc_output(TIM4, TIM_OC1N);
  timer_enable_oc_output(TIM4, TIM_OC2);
  timer_enable_oc_output(TIM4, TIM_OC3);
  timer_enable_oc_output(TIM4, TIM_OC4);
  //timer_enable_break_main_output(TIM4);
  timer_set_oc_value(TIM4, TIM_OC1, 128);
  timer_set_oc_value(TIM4, TIM_OC2, 256);
  timer_set_oc_value(TIM4, TIM_OC3, 512);
  timer_set_oc_value(TIM4, TIM_OC4, 1024);
  timer_set_period(TIM4, 1023);
  timer_enable_counter(TIM4);


   radd_init();
  while(1) {
  }


  //gpio_clear(RADD_GPIO, RADD_PINA);

  /*
  // Button pin
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);

  gpio_setup();

  // 4 pwms
  rcc_periph_clock_enable(RCC_TIM4);
  gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE,
  GPIO12 | GPIO13 | GPIO14 | GPIO15);
  gpio_set_af(GPIOD, GPIO_AF2,
  GPIO12 | GPIO13 | GPIO14 | GPIO15);
  rcc_periph_clock_enable(RCC_TIM4);
  timer_reset(TIM4);
  timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_2,
  TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM4,  120000000 / 2 / (2* 15000 * 1024));
  timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM2);
  timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_PWM2);
  timer_set_oc_mode(TIM4, TIM_OC3, TIM_OCM_PWM2);
  timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM2);
  timer_enable_oc_output(TIM4, TIM_OC1);
  timer_enable_oc_output(TIM4, TIM_OC2);
  timer_enable_oc_output(TIM4, TIM_OC3);
  timer_enable_oc_output(TIM4, TIM_OC4);
  //timer_enable_break_main_output(TIM4);
  timer_set_oc_value(TIM4, TIM_OC1, 512);
  timer_set_oc_value(TIM4, TIM_OC2, 1);
  timer_set_oc_value(TIM4, TIM_OC3, 1);
  timer_set_oc_value(TIM4, TIM_OC4, 1023);
  timer_set_period(TIM4, 1023);
  timer_enable_counter(TIM4);
  // encoder
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1 | GPIO15);
  gpio_set_af(GPIOA, GPIO_AF1, GPIO1 | GPIO15);
  rcc_periph_clock_enable(RCC_TIM2);
  timer_reset(TIM2);
  timer_set_period(TIM2, 512 -1); // number of increments
  // set encoder mode
  // * 0x1 : only one input generate an event
  timer_slave_set_mode(TIM2, 0x1);
  // divide by 2, so the counter reflects the encoder position in increment
  timer_set_prescaler(TIM2, 1);
  timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
  timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI2);

  timer_enable_counter(TIM2);
  nvic_enable_irq(NVIC_TIM2_IRQ);
  timer_enable_irq(TIM2, TIM_DIER_UIE);

  // red led
  gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);

  gpio_set(GPIOD, GPIO14);
  timer_set_oc_value(TIM4, TIM_OC1, 300);

  while (1) {
  counter_enc_update();

  if ( encoderPos.numIncrements <= low ) {
  timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
  gpio_clear(GPIOD, GPIO14);
  }

  if ( encoderPos.numIncrements >= high ) {
  timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM2);
  gpio_set(GPIOD, GPIO14);
  }
  }
  */

  while(1) {}
}


/*
  void tim2_isr(void)
  {
  if (timer_get_flag(TIM2, TIM_DIER_UIE))
  timer_clear_flag(TIM2, TIM_DIER_UIE);

  if ( timer_get_counter(TIM2) )
  encoderPos.numTurns++;
  else
  encoderPos.numTurns--;
  }
*/
