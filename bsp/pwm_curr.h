#ifndef PWM_CURR_H
#define PWM_CURR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32f30x.h"
#include <stdint.h>

// TIMER0 3x-PWM gate drive + ADC phase-current / bus-voltage acquisition.
//
// PWM: TIMER0 center-aligned, CH0/CH1/CH2 -> PA8/PA9/PA10 -> DRV8323 INHA/B/C.
//      The DRV8323 runs in 3x PWM mode, so only the three high-side inputs are
//      driven; the complementary low sides and dead-time are handled inside the
//      DRV. CH3 is a hidden channel used only to trigger the ADC at the PWM peak.
//
// Current sensing: ADC0 + ADC1 in dual inserted-parallel mode, triggered by
//      TIMER0_CH3, sample the three shunt currents simultaneously at the centre
//      of the PWM period (all low sides on). The inserted end-of-conversion
//      interrupt (ADC0_1_IRQHandler) is the FOC control-loop tick.
//        IA = PA3 (ADC0 inserted 0), IB = PA2 (ADC0 inserted 1),
//        IC = PA1 (ADC1 inserted 0).
// Monitoring: ADC0 regular group + DMA0_CH0 continuously samples VBUS (PA0) and
//      two spare channels into adc_buff[].

#define PWM_FREQUENCY          20000U
#define TIMER0_CLK_MHZ         120U
#define PWM_PERIOD_CYCLES      ((uint16_t)((TIMER0_CLK_MHZ * 1000000U / PWM_FREQUENCY) & 0xFFFEU))
#define HALF_PWM_PERIOD_CYCLES ((uint16_t)(PWM_PERIOD_CYCLES / 2U))  // = TIMER0 period (center-aligned)

// Current/voltage scaling. I_SCALE must match the DRV8323 CSA gain configured in
// drv8323.c (DRV_CFG_CSA_GAIN = 20 V/V). VBUS divider is 18k:1k -> ratio 19.
#define ADC_VREF               3.3f
#define ADC_FULL_SCALE         4095.0f
#define SHUNT_OHM              0.005f
#define CSA_GAIN               20.0f
#define I_SCALE                (ADC_VREF / ADC_FULL_SCALE / (CSA_GAIN * SHUNT_OHM))
#define V_SCALE                (19.0f * ADC_VREF / ADC_FULL_SCALE)

// Inserted data registers hold the simultaneously sampled phase currents.
#define READ_IA_ADC()          ((uint16_t)ADC_IDATA0(ADC0))  // PA3
#define READ_IB_ADC()          ((uint16_t)ADC_IDATA1(ADC0))  // PA2
#define READ_IC_ADC()          ((uint16_t)ADC_IDATA0(ADC1))  // PA1

// adc_buff: [0] = VBUS, [1] = spare (PB0), [2] = spare (PB1). Module-owned;
// read-only to other modules.
extern uint16_t adc_buff[3];
extern int16_t ia_adc_offset;
extern int16_t ib_adc_offset;
extern int16_t ic_adc_offset;

void pwm_curr_init(void);

// Register the function called from the ADC inserted-conversion ISR (the FOC
// current loop). Pass NULL to detach.
void pwm_curr_attach_callback(void (*cb)(void));

// Set the three high-side compare values (0..HALF_PWM_PERIOD_CYCLES).
void pwm_set_duty(uint16_t ch0, uint16_t ch1, uint16_t ch2);

// Enable / disable the three channel outputs (gate drive). The timer keeps
// running (and triggering the ADC) regardless.
void pwm_enable(void);
void pwm_disable(void);

// Average the phase-current ADC channels with the motor at rest to capture the
// zero-current offsets. Returns 0 on success, -1 if any offset is out of range.
int pwm_curr_calibrate_offset(void);

static inline float read_vbus(void) {
  return (float)adc_buff[0] * V_SCALE;
}

static inline float read_ia(void) {
  return (float)(READ_IA_ADC() - ia_adc_offset) * I_SCALE;
}

static inline float read_ib(void) {
  return (float)(READ_IB_ADC() - ib_adc_offset) * I_SCALE;
}

static inline float read_ic(void) {
  return (float)(READ_IC_ADC() - ic_adc_offset) * I_SCALE;
}

#ifdef __cplusplus
}
#endif

#endif /* PWM_CURR_H */
