

#include "delay.h"

void delay_init(void) {
  rcc_periph_clock_enable(RCC_TIM2);
  rcc_periph_reset_pulse(RST_TIM2);

  // counter freq = fckpsc / prescaler + 1
  // 16mHz / 16 => 1mHz counter (1us ticks)
  timer_set_prescaler(TIM2, 63);

  // prescaler is not updated until a timer event occurs
  timer_generate_event(TIM2, TIM_EGR_UG);
}

void delay_us(uint32_t us) {
  // this function call overhead takes about 4us so subtract that from the delay
  if (us >= 4) {
    us -= 4;
  }

  timer_set_counter(TIM2, 0);
  timer_enable_counter(TIM2);
  while (timer_get_counter(TIM2) < us);
  timer_disable_counter(TIM2);
}
