#include "drv8323.h"

#include "gd32f30x.h"

// --- Pin mapping ---
#define DRV_SPI            SPI1
#define DRV_SPI_PORT       GPIOB
#define DRV_SCK_PIN        GPIO_PIN_13
#define DRV_MISO_PIN       GPIO_PIN_14
#define DRV_MOSI_PIN       GPIO_PIN_15
#define DRV_CS_PORT        GPIOB
#define DRV_CS_PIN         GPIO_PIN_12
#define DRV_EN_PORT        GPIOB
#define DRV_EN_PIN         GPIO_PIN_7
#define DRV_FAULT_PORT     GPIOB
#define DRV_FAULT_PIN      GPIO_PIN_2

// --- SPI frame format: bit15 = R/W (1=read), bits14:11 = addr, bits10:0 = data ---
#define DRV_FRAME_READ     (1U << 15)
#define DRV_ADDR_Pos       11
#define DRV_DATA_MASK      0x07FFU

// --- Register field positions ---
#define DRV_DRIVER_PWM_MODE_Pos 5

#define DRV_GATE_LOCK_Pos       8
#define DRV_GATE_IDRIVEP_Pos    4
#define DRV_GATE_IDRIVEN_Pos    0
#define DRV_GATE_LS_CBC         (1U << 10)
#define DRV_GATE_LS_TDRIVE_Pos  8

#define DRV_OCP_TRETRY          (1U << 10)
#define DRV_OCP_DEAD_TIME_Pos   8
#define DRV_OCP_MODE_Pos        6
#define DRV_OCP_DEG_Pos         4
#define DRV_OCP_VDS_LVL_Pos     0

#define DRV_CSA_FET             (1U << 10)
#define DRV_CSA_VREF_DIV        (1U << 9)
#define DRV_CSA_LS_REF          (1U << 8)
#define DRV_CSA_GAIN_Pos        6
#define DRV_CSA_DIS_SEN         (1U << 5)
#define DRV_CSA_CAL_A           (1U << 4)
#define DRV_CSA_CAL_B           (1U << 3)
#define DRV_CSA_CAL_C           (1U << 2)
#define DRV_CSA_SEN_LVL_Pos     0

#define DRV_DRIVER_OTW_REP      (1U << 7)  // report overtemp warning on nFAULT
#define DRV_DRIVER_CTRL_CLR_FLT (1U << 0)
#define DRV_GATE_LOCK_UNLOCK    0x3U  // 011b: registers writable

// --- Board configuration (ASDM40N80 power stage, Rds(on) 7.34 mOhm typ /
//     9.0 mOhm max; 5 mOhm shunt; 24 V bus; motor 2.9 A rated / 10.36 A stall) ---
#define DRV_CFG_PWM_MODE   DRV8323_PWM_MODE_3X
#define DRV_CFG_IDRIVEP_HS 0x4U  // 120 mA source, high side
#define DRV_CFG_IDRIVEN_HS 0x4U  // 240 mA sink, high side
#define DRV_CFG_IDRIVEP_LS 0x4U  // 120 mA source, low side
#define DRV_CFG_IDRIVEN_LS 0x4U  // 240 mA sink, low side
#define DRV_CFG_TDRIVE     0x3U  // 4000 ns peak gate-current drive time
#define DRV_CFG_DEAD_TIME  0x2U  // 200 ns dead time (inserted by DRV in 3x mode)
#define DRV_CFG_OCP_MODE   0x0U  // 00b = latched shutdown (MCU manages recovery)
#define DRV_CFG_OCP_DEG    0x1U  // 4 us deglitch
// VDS overcurrent trip = VDS_LVL / Rds(on). 0011b = 0.26 V -> ~29 A at hot
// Rds(on)=9 mOhm (~35 A typ), i.e. ~2.8x the 10.36 A stall current: a hard-
// short protection that will not nuisance-trip during normal operation.
#define DRV_CFG_VDS_LVL    0x3U  // 0.26 V VDS overcurrent threshold
#define DRV_CFG_CSA_GAIN   DRV8323_CSA_GAIN_20
// Sense overcurrent trip = SEN_LVL / Rshunt. 00b = 0.25 V / 5 mOhm = 50 A,
// a coarse backstop well above the measurable range.
#define DRV_CFG_SEN_LVL    0x0U  // 0.25 V sense overcurrent threshold

#define DRV_WAKE_DELAY_US  2000U  // tWAKE after EN rising edge

static void drv_delay_us(uint32_t us) {
  // ~120 MHz core; conservative so the real delay is never shorter than asked.
  volatile uint32_t n = us * 30U;
  while (n-- > 0U) {
    __NOP();
  }
}

static void drv_gpio_config(void) {
  rcu_periph_clock_enable(RCU_GPIOA);  // unused here but keeps AF clock domain ready
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_AF);

  // SPI1: SCK + MOSI as AF push-pull, MISO as floating input.
  gpio_init(DRV_SPI_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, DRV_SCK_PIN | DRV_MOSI_PIN);
  gpio_init(DRV_SPI_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, DRV_MISO_PIN);

  // nSCS managed manually as a GPIO; idle high.
  gpio_init(DRV_CS_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, DRV_CS_PIN);
  gpio_bit_set(DRV_CS_PORT, DRV_CS_PIN);

  // EN output, start low (device asleep); nFAULT input (external pull-up).
  gpio_init(DRV_EN_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, DRV_EN_PIN);
  gpio_bit_reset(DRV_EN_PORT, DRV_EN_PIN);
  gpio_init(DRV_FAULT_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, DRV_FAULT_PIN);
}

static void drv_spi_config(void) {
  spi_parameter_struct spi;

  rcu_periph_clock_enable(RCU_SPI1);
  spi_i2s_deinit(DRV_SPI);
  spi_struct_para_init(&spi);

  spi.device_mode = SPI_MASTER;
  spi.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
  spi.frame_size = SPI_FRAMESIZE_16BIT;          // one DRV word per transfer
  spi.nss = SPI_NSS_SOFT;
  spi.endian = SPI_ENDIAN_MSB;
  spi.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;  // SPI mode 1: CPOL=0, CPHA=1
  spi.prescale = SPI_PSC_16;                     // 60 MHz / 16 = 3.75 MHz < 10 MHz max
  spi_init(DRV_SPI, &spi);

  spi_nss_internal_high(DRV_SPI);  // keep internal NSS high to avoid mode fault
  spi_enable(DRV_SPI);
}

// One 16-bit SPI transaction with manual chip-select framing.
static uint16_t drv_xfer(uint16_t word) {
  gpio_bit_reset(DRV_CS_PORT, DRV_CS_PIN);
  __NOP();
  __NOP();
  __NOP();
  __NOP();

  while (spi_i2s_flag_get(DRV_SPI, SPI_FLAG_TBE) == RESET) {}
  spi_i2s_data_transmit(DRV_SPI, word);
  while (spi_i2s_flag_get(DRV_SPI, SPI_FLAG_RBNE) == RESET) {}
  uint16_t rx = spi_i2s_data_receive(DRV_SPI);
  while (spi_i2s_flag_get(DRV_SPI, SPI_FLAG_TRANS) != RESET) {}

  gpio_bit_set(DRV_CS_PORT, DRV_CS_PIN);
  drv_delay_us(1U);  // >= 400 ns required between words
  return rx;
}

uint16_t drv8323_read_reg(uint8_t addr) {
  uint16_t word = DRV_FRAME_READ | ((uint16_t)(addr & 0x0FU) << DRV_ADDR_Pos);
  return drv_xfer(word) & DRV_DATA_MASK;
}

void drv8323_write_reg(uint8_t addr, uint16_t data) {
  uint16_t word = ((uint16_t)(addr & 0x0FU) << DRV_ADDR_Pos) | (data & DRV_DATA_MASK);
  (void)drv_xfer(word);
}

// Base value of the Driver Control register (PWM mode + OTW reporting), without
// the self-clearing CLR_FLT bit.
static inline uint16_t drv_driver_ctrl_base(void) {
  return (uint16_t)((DRV_CFG_PWM_MODE << DRV_DRIVER_PWM_MODE_Pos) | DRV_DRIVER_OTW_REP);
}

static inline uint16_t drv_ocp_ctrl_cfg(void) {
  return (uint16_t)((DRV_CFG_DEAD_TIME << DRV_OCP_DEAD_TIME_Pos)
                    | (DRV_CFG_OCP_MODE << DRV_OCP_MODE_Pos)
                    | (DRV_CFG_OCP_DEG << DRV_OCP_DEG_Pos)
                    | (DRV_CFG_VDS_LVL << DRV_OCP_VDS_LVL_Pos));
}

static inline uint16_t drv_csa_ctrl_cfg(void) {
  return (uint16_t)(DRV_CSA_VREF_DIV  // bidirectional (VREF/2 reference)
                    | (DRV_CFG_CSA_GAIN << DRV_CSA_GAIN_Pos)
                    | (DRV_CFG_SEN_LVL << DRV_CSA_SEN_LVL_Pos));
}

void drv8323_apply_config(void) {
  uint16_t gate_hs = (uint16_t)((DRV_GATE_LOCK_UNLOCK << DRV_GATE_LOCK_Pos)
                                | (DRV_CFG_IDRIVEP_HS << DRV_GATE_IDRIVEP_Pos)
                                | (DRV_CFG_IDRIVEN_HS << DRV_GATE_IDRIVEN_Pos));

  uint16_t gate_ls = (uint16_t)(DRV_GATE_LS_CBC
                                | (DRV_CFG_TDRIVE << DRV_GATE_LS_TDRIVE_Pos)
                                | (DRV_CFG_IDRIVEP_LS << DRV_GATE_IDRIVEP_Pos)
                                | (DRV_CFG_IDRIVEN_LS << DRV_GATE_IDRIVEN_Pos));

  drv8323_write_reg(DRV8323_REG_DRIVER_CTRL, drv_driver_ctrl_base());
  drv8323_write_reg(DRV8323_REG_GATE_DRIVE_HS, gate_hs);
  drv8323_write_reg(DRV8323_REG_GATE_DRIVE_LS, gate_ls);
  drv8323_write_reg(DRV8323_REG_OCP_CTRL, drv_ocp_ctrl_cfg());
  drv8323_write_reg(DRV8323_REG_CSA_CTRL, drv_csa_ctrl_cfg());
}

bool drv8323_init(void) {
  drv_gpio_config();
  drv_spi_config();

  drv8323_enable();
  drv_delay_us(DRV_WAKE_DELAY_US);

  drv8323_clear_fault();
  drv8323_apply_config();

  // Verify communication: read back the configuration registers and confirm
  // they match what was written (CLR_FLT auto-clears, so DRIVER_CTRL reads back
  // as its base value). A mismatch means the SPI link to the DRV is broken.
  bool ok = true;
  ok &= (drv8323_read_reg(DRV8323_REG_DRIVER_CTRL) == drv_driver_ctrl_base());
  ok &= (drv8323_read_reg(DRV8323_REG_OCP_CTRL) == drv_ocp_ctrl_cfg());
  ok &= (drv8323_read_reg(DRV8323_REG_CSA_CTRL) == drv_csa_ctrl_cfg());
  return ok;
}

void drv8323_enable(void) {
  gpio_bit_set(DRV_EN_PORT, DRV_EN_PIN);
}

void drv8323_disable(void) {
  gpio_bit_reset(DRV_EN_PORT, DRV_EN_PIN);
}

bool drv8323_fault_active(void) {
  return gpio_input_bit_get(DRV_FAULT_PORT, DRV_FAULT_PIN) == RESET;
}

uint16_t drv8323_get_fault_status1(void) {
  return drv8323_read_reg(DRV8323_REG_FAULT_STATUS1);
}

uint16_t drv8323_get_vgs_status2(void) {
  return drv8323_read_reg(DRV8323_REG_VGS_STATUS2);
}

void drv8323_clear_fault(void) {
  drv8323_write_reg(DRV8323_REG_DRIVER_CTRL,
                    drv_driver_ctrl_base() | DRV_DRIVER_CTRL_CLR_FLT);
}

void drv8323_parse_faults(drv8323_faults_t *f) {
  uint16_t fs1 = drv8323_read_reg(DRV8323_REG_FAULT_STATUS1);
  uint16_t fs2 = drv8323_read_reg(DRV8323_REG_VGS_STATUS2);

  f->fs1_raw = fs1;
  f->fs2_raw = fs2;

  f->vds_ocp = (fs1 & DRV8323_FS1_VDS_OCP) != 0U;
  f->gdf     = (fs1 & DRV8323_FS1_GDF) != 0U;
  f->uvlo    = (fs1 & DRV8323_FS1_UVLO) != 0U;
  f->otsd    = (fs1 & DRV8323_FS1_OTSD) != 0U;
  f->vds_phase = (uint8_t)(fs1 & 0x3FU);  // VDS_HA..VDS_LC

  f->sen_ocp = (fs2 & (DRV8323_VGS_SA_OC | DRV8323_VGS_SB_OC | DRV8323_VGS_SC_OC)) != 0U;
  f->otw     = (fs2 & DRV8323_VGS_OTW) != 0U;
  f->cpuv    = (fs2 & DRV8323_VGS_CPUV) != 0U;
  f->vgs_phase = (uint8_t)(fs2 & 0x3FU);  // VGS_HA..VGS_LC
  f->sen_phase = (uint8_t)((fs2 >> 8) & 0x7U);  // SA/SB/SC_OC
}

// --- nFAULT EXTI ---
static volatile bool s_nfault_pending = false;
static void (*s_emergency_cb)(void) = 0;

void drv8323_attach_emergency_cb(void (*cb)(void)) {
  s_emergency_cb = cb;
}

bool drv8323_nfault_edge_pending(void) {
  bool p = s_nfault_pending;
  s_nfault_pending = false;
  return p;
}

void drv8323_fault_exti_init(void) {
  rcu_periph_clock_enable(RCU_AF);
  // PB2 = EXTI line 2, falling edge (nFAULT is active-low).
  gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_2);
  exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
  exti_interrupt_flag_clear(EXTI_2);
  // High preemption priority so the emergency stop pre-empts most work.
  nvic_irq_enable(EXTI2_IRQn, 1U, 0U);
}

void drv8323_exti_isr(void) {
  if (exti_interrupt_flag_get(EXTI_2) != RESET) {
    exti_interrupt_flag_clear(EXTI_2);
    s_nfault_pending = true;
    if (s_emergency_cb != 0) {
      s_emergency_cb();  // must be short, no SPI
    }
  }
}

void drv8323_set_csa_gain(drv8323_csa_gain_t gain) {
  uint16_t csa = drv8323_read_reg(DRV8323_REG_CSA_CTRL);
  csa &= (uint16_t)~(0x3U << DRV_CSA_GAIN_Pos);
  csa |= (uint16_t)((uint16_t)gain << DRV_CSA_GAIN_Pos);
  drv8323_write_reg(DRV8323_REG_CSA_CTRL, csa);
}

void drv8323_csa_calibrate(bool enable) {
  uint16_t csa = drv8323_read_reg(DRV8323_REG_CSA_CTRL);
  if (enable) {
    csa |= (DRV_CSA_CAL_A | DRV_CSA_CAL_B | DRV_CSA_CAL_C);
  } else {
    csa &= (uint16_t)~(DRV_CSA_CAL_A | DRV_CSA_CAL_B | DRV_CSA_CAL_C);
  }
  drv8323_write_reg(DRV8323_REG_CSA_CTRL, csa);
}
