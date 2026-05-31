#include "pwm_curr.h"

// Provided by main.c (busy-wait millisecond delay, used only at init).
extern void delay_ms(uint16_t ms);

uint16_t adc_buff[3];
int16_t ia_adc_offset = 2048;
int16_t ib_adc_offset = 2048;
int16_t ic_adc_offset = 2048;

static void (*s_adc_callback)(void) = 0;

// CH3 fires the ADC trigger a few cycles before the counter peak, so the
// inserted sampling lands at the centre of the PWM (all low sides conducting).
#define ADC_TRIGGER_OFFSET 5U

static void pwm_curr_rcu_config(void) {
  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(RCU_TIMER0);
  rcu_periph_clock_enable(RCU_ADC0);
  rcu_periph_clock_enable(RCU_ADC1);
  rcu_periph_clock_enable(RCU_DMA0);

  // ADC clock = APB2 / 4 = 30 MHz.
  rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
}

static void pwm_curr_gpio_config(void) {
  // Phase-current and bus-voltage inputs (analog).
  gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0);  // VBUS  - ADC0_IN0
  gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_1);  // IC    - ADC1_IN1
  gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_2);  // IB    - ADC0_IN2
  gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_3);  // IA    - ADC0_IN3
  gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0);  // spare - ADC0_IN8
  gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_1);  // spare - ADC0_IN9

  // High-side PWM outputs to DRV8323 INHA/INHB/INHC.
  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);   // CH0 - INHA
  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);   // CH1 - INHB
  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);  // CH2 - INHC
}

static void pwm_curr_timer_config(void) {
  timer_parameter_struct timer_initpara;
  timer_oc_parameter_struct timer_ocinitpara;
  timer_break_parameter_struct timer_breakpara;

  timer_deinit(TIMER0);
  timer_struct_para_init(&timer_initpara);
  timer_initpara.prescaler = 0U;
  timer_initpara.alignedmode = TIMER_COUNTER_CENTER_UP;
  timer_initpara.counterdirection = TIMER_COUNTER_UP;
  timer_initpara.period = HALF_PWM_PERIOD_CYCLES;
  timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
  timer_initpara.repetitioncounter = 0U;
  timer_init(TIMER0, &timer_initpara);

  timer_channel_output_struct_para_init(&timer_ocinitpara);
  timer_ocinitpara.outputstate = TIMER_CCX_ENABLE;
  timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;  // 3x PWM: no complementary outputs
  timer_ocinitpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
  timer_ocinitpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
  timer_ocinitpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
  timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
  timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocinitpara);
  timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocinitpara);
  timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocinitpara);
  timer_channel_output_config(TIMER0, TIMER_CH_3, &timer_ocinitpara);

  // CH0/CH1/CH2: PWM0 mode, shadowed compare for glitch-free duty updates.
  timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0U);
  timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
  timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
  timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0U);
  timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
  timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);
  timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, 0U);
  timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);
  timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);

  // CH3: ADC trigger near the counter peak (PWM1 mode, no shadow).
  timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3,
                                          (uint16_t)(HALF_PWM_PERIOD_CYCLES - ADC_TRIGGER_OFFSET));
  timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM1);
  timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

  // Dead-time is generated inside the DRV8323 (3x PWM mode), so the timer break
  // unit only provides the default disabled state.
  timer_break_struct_para_init(&timer_breakpara);
  timer_breakpara.runoffstate = TIMER_ROS_STATE_DISABLE;
  timer_breakpara.ideloffstate = TIMER_IOS_STATE_DISABLE;
  timer_breakpara.deadtime = 0U;
  timer_breakpara.breakpolarity = TIMER_BREAK_POLARITY_LOW;
  timer_breakpara.outputautostate = TIMER_OUTAUTO_DISABLE;
  timer_breakpara.protectmode = TIMER_CCHP_PROT_OFF;
  timer_breakpara.breakstate = TIMER_BREAK_DISABLE;
  timer_break_config(TIMER0, &timer_breakpara);

  // Hold the counter when the core halts on a breakpoint (keeps the FETs safe).
  dbg_periph_enable(DBG_TIMER0_HOLD);

  timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
  timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
  timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
  timer_primary_output_config(TIMER0, ENABLE);

  timer_enable(TIMER0);
}

static void pwm_curr_dma_config(void) {
  dma_parameter_struct dma;

  // ADC0 regular group -> adc_buff, circular.
  dma_deinit(DMA0, DMA_CH0);
  dma.periph_addr = (uint32_t)&ADC_RDATA(ADC0);
  dma.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma.memory_addr = (uint32_t)adc_buff;
  dma.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
  dma.memory_width = DMA_MEMORY_WIDTH_16BIT;
  dma.direction = DMA_PERIPHERAL_TO_MEMORY;
  dma.number = 3U;
  dma.priority = DMA_PRIORITY_HIGH;
  dma_init(DMA0, DMA_CH0, &dma);
  dma_circulation_enable(DMA0, DMA_CH0);
  dma_channel_enable(DMA0, DMA_CH0);
}

static void pwm_curr_adc_config(void) {
  adc_deinit(ADC0);
  adc_mode_config(ADC_DAUL_INSERTED_PARALLEL);

  // --- ADC0: inserted currents (IA, IB) + regular monitoring (VBUS, spares) ---
  adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
  adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
  adc_special_function_config(ADC0, ADC_INSERTED_CHANNEL_AUTO, DISABLE);
  adc_resolution_config(ADC0, ADC_RESOLUTION_12B);
  adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
  adc_dma_mode_enable(ADC0);

  adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 2U);
  adc_inserted_channel_config(ADC0, 0U, ADC_CHANNEL_3, ADC_SAMPLETIME_7POINT5);  // PA3 = IA
  adc_inserted_channel_config(ADC0, 1U, ADC_CHANNEL_2, ADC_SAMPLETIME_7POINT5);  // PA2 = IB
  adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_CH3);
  adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);

  adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 3U);
  adc_regular_channel_config(ADC0, 0U, ADC_CHANNEL_0, ADC_SAMPLETIME_7POINT5);  // PA0 = VBUS
  adc_regular_channel_config(ADC0, 1U, ADC_CHANNEL_8, ADC_SAMPLETIME_7POINT5);  // PB0 = spare
  adc_regular_channel_config(ADC0, 2U, ADC_CHANNEL_9, ADC_SAMPLETIME_7POINT5);  // PB1 = spare
  adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
  adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);

  // --- ADC1: inserted IC, triggered in parallel with ADC0 ---
  adc_deinit(ADC1);
  adc_special_function_config(ADC1, ADC_SCAN_MODE, DISABLE);
  adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, DISABLE);
  adc_special_function_config(ADC1, ADC_INSERTED_CHANNEL_AUTO, DISABLE);
  adc_resolution_config(ADC1, ADC_RESOLUTION_12B);
  adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
  adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL, 1U);
  adc_inserted_channel_config(ADC1, 0U, ADC_CHANNEL_1, ADC_SAMPLETIME_7POINT5);  // PA1 = IC
  adc_external_trigger_source_config(ADC1, ADC_INSERTED_CHANNEL, ADC0_1_2_EXTTRIG_INSERTED_NONE);
  adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, ENABLE);

  // Bring up ADC0.
  adc_enable(ADC0);
  delay_ms(1);
  adc_calibration_enable(ADC0);
  adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);

  // Bring up ADC1.
  adc_enable(ADC1);
  delay_ms(1);
  adc_calibration_enable(ADC1);
  adc_software_trigger_enable(ADC1, ADC_INSERTED_CHANNEL);

  // Inserted end-of-conversion drives the FOC loop.
  adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
  adc_interrupt_enable(ADC0, ADC_INT_EOIC);
  nvic_irq_enable(ADC0_1_IRQn, 0U, 0U);
}

void pwm_curr_init(void) {
  pwm_curr_rcu_config();
  pwm_curr_gpio_config();
  pwm_curr_timer_config();
  pwm_curr_dma_config();
  pwm_curr_adc_config();

  pwm_set_duty(HALF_PWM_PERIOD_CYCLES / 2U, HALF_PWM_PERIOD_CYCLES / 2U,
               HALF_PWM_PERIOD_CYCLES / 2U);
}

void pwm_curr_attach_callback(void (*cb)(void)) {
  s_adc_callback = cb;
}

void pwm_set_duty(uint16_t ch0, uint16_t ch1, uint16_t ch2) {
  TIMER_CH0CV(TIMER0) = ch0;
  TIMER_CH1CV(TIMER0) = ch1;
  TIMER_CH2CV(TIMER0) = ch2;
}

void pwm_enable(void) {
  timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_ENABLE);
  timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_ENABLE);
  timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_ENABLE);
}

void pwm_disable(void) {
  timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
  timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
  timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
}

int pwm_curr_calibrate_offset(void) {
  int32_t sum_a = 0;
  int32_t sum_b = 0;
  int32_t sum_c = 0;
  const int samples = 64;

  timer_flag_clear(TIMER0, TIMER_FLAG_UP);
  while (timer_flag_get(TIMER0, TIMER_FLAG_UP) == RESET) {}
  timer_flag_clear(TIMER0, TIMER_FLAG_UP);

  for (int i = 0; i < samples;) {
    if (timer_flag_get(TIMER0, TIMER_FLAG_UP) == SET) {
      timer_flag_clear(TIMER0, TIMER_FLAG_UP);
      sum_a += READ_IA_ADC();
      sum_b += READ_IB_ADC();
      sum_c += READ_IC_ADC();
      i++;
    }
  }

  ia_adc_offset = (int16_t)(sum_a / samples);
  ib_adc_offset = (int16_t)(sum_b / samples);
  ic_adc_offset = (int16_t)(sum_c / samples);

  // Offsets should sit near mid-scale (bidirectional CSA, VREF/2).
  const int16_t mid = 2048;
  const int16_t tol = 200;
  if (ia_adc_offset < mid - tol || ia_adc_offset > mid + tol) {
    return -1;
  }
  if (ib_adc_offset < mid - tol || ib_adc_offset > mid + tol) {
    return -1;
  }
  if (ic_adc_offset < mid - tol || ic_adc_offset > mid + tol) {
    return -1;
  }
  return 0;
}

void ADC0_1_IRQHandler(void) {
  ADC_STAT(ADC0) &= ~((uint32_t)ADC_INT_FLAG_EOIC);

  if (s_adc_callback != 0) {
    s_adc_callback();
  }
}
