#ifndef DRV8323_H
#define DRV8323_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

// DRV8323RS 3-phase gate driver, SPI variant with integrated current sense.
// Wiring (see doc/原理图.pdf and pinmap):
//   SPI1: SCK=PB13, MISO=PB14, MOSI=PB15, nSCS=PB12 (software GPIO)
//   DRV_EN  = PB7  (output)
//   nFAULT  = PB2  (input, external 10k pull-up)
// The board ties INLA/INLB/INLC high and drives only INHA/INHB/INHC from
// TIMER0, so the device must run in 3x PWM mode (configured in drv8323_init).

// SPI register addresses (4-bit).
#define DRV8323_REG_FAULT_STATUS1 0x00U  // read-only
#define DRV8323_REG_VGS_STATUS2   0x01U  // read-only
#define DRV8323_REG_DRIVER_CTRL   0x02U
#define DRV8323_REG_GATE_DRIVE_HS 0x03U
#define DRV8323_REG_GATE_DRIVE_LS 0x04U
#define DRV8323_REG_OCP_CTRL      0x05U
#define DRV8323_REG_CSA_CTRL      0x06U

// Fault Status 1 (reg 0x00) bit masks.
#define DRV8323_FS1_FAULT   (1U << 10)  // any fault
#define DRV8323_FS1_VDS_OCP (1U << 9)   // VDS overcurrent
#define DRV8323_FS1_GDF     (1U << 8)   // gate drive fault
#define DRV8323_FS1_UVLO    (1U << 7)   // undervoltage lockout
#define DRV8323_FS1_OTSD    (1U << 6)   // overtemperature shutdown
#define DRV8323_FS1_VDS_HA  (1U << 5)
#define DRV8323_FS1_VDS_LA  (1U << 4)
#define DRV8323_FS1_VDS_HB  (1U << 3)
#define DRV8323_FS1_VDS_LB  (1U << 2)
#define DRV8323_FS1_VDS_HC  (1U << 1)
#define DRV8323_FS1_VDS_LC  (1U << 0)

// VGS Status 2 (reg 0x01) bit masks.
#define DRV8323_VGS_SA_OC   (1U << 10)  // sense A overcurrent
#define DRV8323_VGS_SB_OC   (1U << 9)
#define DRV8323_VGS_SC_OC   (1U << 8)
#define DRV8323_VGS_OTW     (1U << 7)   // overtemperature warning
#define DRV8323_VGS_CPUV    (1U << 6)   // charge pump undervoltage
#define DRV8323_VGS_VGS_HA  (1U << 5)
#define DRV8323_VGS_VGS_LA  (1U << 4)
#define DRV8323_VGS_VGS_HB  (1U << 3)
#define DRV8323_VGS_VGS_LB  (1U << 2)
#define DRV8323_VGS_VGS_HC  (1U << 1)
#define DRV8323_VGS_VGS_LC  (1U << 0)

// PWM_MODE field values (DRIVER_CTRL bits 6:5).
#define DRV8323_PWM_MODE_6X    0U
#define DRV8323_PWM_MODE_3X    1U
#define DRV8323_PWM_MODE_1X    2U
#define DRV8323_PWM_MODE_INDEP 3U

typedef enum {
  DRV8323_CSA_GAIN_5  = 0U,
  DRV8323_CSA_GAIN_10 = 1U,
  DRV8323_CSA_GAIN_20 = 2U,
  DRV8323_CSA_GAIN_40 = 3U,
} drv8323_csa_gain_t;

// Decoded fault state from the two status registers. The *_phase fields carry
// the low 6 bits of the corresponding register (VDS_xx / VGS_xx, bit5..bit0 =
// HA,LA,HB,LB,HC,LC) so the offending MOSFET can be pinpointed.
typedef struct {
  bool vds_ocp;     // any MOSFET VDS overcurrent
  bool sen_ocp;     // any sense-resistor overcurrent
  bool gdf;         // gate-drive fault
  bool uvlo;        // VM undervoltage lockout
  bool cpuv;        // charge-pump undervoltage
  bool otsd;        // overtemperature shutdown
  bool otw;         // overtemperature warning
  uint8_t vds_phase;  // FS1 bits 5:0
  uint8_t vgs_phase;  // FS2 bits 5:0
  uint8_t sen_phase;  // FS2 bits 10:8 (SA/SB/SC_OC)
  uint16_t fs1_raw;   // Fault Status 1 (0x00)
  uint16_t fs2_raw;   // Fault Status 2 (0x01)
} drv8323_faults_t;

// Bring up the device: GPIO + SPI1, assert EN, then write the configuration
// registers for 3x PWM mode. Returns true if the SPI read-back matches what
// was written (i.e. communication with the DRV8323 is verified).
bool drv8323_init(void);

// EN pin control. disable() puts the device in low-power sleep and clears
// any latched faults on the next enable() (ENABLE reset pulse).
void drv8323_enable(void);
void drv8323_disable(void);

// Raw register access. addr is 4-bit; data/return value are 11-bit.
uint16_t drv8323_read_reg(uint8_t addr);
void drv8323_write_reg(uint8_t addr, uint16_t data);

// (Re)write all configuration registers (PWM mode, gate drive, OCP, CSA). The
// DRV resets its registers on a UVLO/sleep, so call this when recovering from a
// fault before clearing it. Also used internally by drv8323_init().
void drv8323_apply_config(void);

// True while the nFAULT pin is asserted (low).
bool drv8323_fault_active(void);

uint16_t drv8323_get_fault_status1(void);
uint16_t drv8323_get_vgs_status2(void);

// Read both status registers over SPI and decode them into `f`.
void drv8323_parse_faults(drv8323_faults_t *f);

// Clear latched fault bits via the CLR_FLT bit (self-clearing).
void drv8323_clear_fault(void);

// --- nFAULT EXTI (hardware emergency stop) ---
// Configure PB2/nFAULT as a falling-edge EXTI interrupt. Call AFTER the PWM
// timer is initialised (the registered emergency callback disables the PWM).
void drv8323_fault_exti_init(void);

// Register the callback invoked from the nFAULT EXTI ISR. It runs in interrupt
// context and must be short (e.g. disable the PWM outputs); it must NOT touch
// the SPI bus. Pass NULL to detach.
void drv8323_attach_emergency_cb(void (*cb)(void));

// Returns (and clears) whether an nFAULT falling edge has occurred since the
// last call. The slow monitor uses this to decide when to read the status regs.
bool drv8323_nfault_edge_pending(void);

// Called from EXTI2_IRQHandler (in app/gd32f30x_it.c).
void drv8323_exti_isr(void);

void drv8323_set_csa_gain(drv8323_csa_gain_t gain);

// Short the CSA inputs so the MCU ADC can measure amplifier offset. Pass true
// to enter calibration, false to return to normal sensing.
void drv8323_csa_calibrate(bool enable);

#ifdef __cplusplus
}
#endif

#endif /* DRV8323_H */
