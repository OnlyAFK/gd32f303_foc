#include "tle5012b.h"

#include "gd32f30x.h"

// --- Pin mapping ---
#define TLE_SPI            SPI2
#define TLE_SPI_PORT       GPIOB
#define TLE_SCK_PIN        GPIO_PIN_3
#define TLE_MISO_PIN       GPIO_PIN_4
#define TLE_MOSI_PIN       GPIO_PIN_5
#define TLE_CS_PORT        GPIOA
#define TLE_CS_PIN         GPIO_PIN_15

// SPI2 DMA mapping is fixed in hardware: RX = DMA1 channel0, TX = DMA1 channel1.
#define TLE_DMA            DMA1
#define TLE_DMA_RX_CH      DMA_CH0
#define TLE_DMA_TX_CH      DMA_CH1
#define TLE_DMA_RX_IRQn    DMA1_Channel0_IRQn

// --- SSC command word: RW[15] Lock[14:11] UPD[10] ADDR[9:4] ND[3:0] ---
// Read AVAL (angle value, address 0x02), ND=0 -> one data word, no safety word.
#define TLE_CMD_READ_AVAL  0x8020U
#define TLE_DUMMY          0xFFFFU

#define TLE_ANGLE_MASK     0x7FFFU      // 15-bit angle field
#define TLE_ANGLE_RES      32768.0f     // 2^15 counts per revolution
#define TLE_2PI            6.28318530718f

#define TLE_READ_TIMEOUT   100000U

// PB5 (MOSI) shares the single bidirectional DATA line with PB4 (MISO) through
// a 100R series resistor. During the data phase the MOSI push-pull driver must
// be released to floating input, otherwise it overpowers the sensor and the
// line is read as constant 0xFFFF. PB5 lives in GPIO_CTL0 bits [23:20]:
//   AF push-pull 50MHz : CNF=10, MODE=11 -> 0xB
//   floating input     : CNF=01, MODE=00 -> 0x4
#define PB5_AS_AF()    (GPIO_CTL0(GPIOB) = (GPIO_CTL0(GPIOB) & ~(0xFU << 20)) | (0xBU << 20))
#define PB5_AS_INPUT() (GPIO_CTL0(GPIOB) = (GPIO_CTL0(GPIOB) & ~(0xFU << 20)) | (0x4U << 20))

static uint16_t tx_dummy = TLE_DUMMY;
static uint16_t rx_word;
static volatile bool read_busy;
static volatile uint16_t angle_raw;
static volatile uint16_t dbg_cmd_echo;
static void (*s_done_callback)(void) = 0;

static void tle_delay_us(uint32_t us) {
  volatile uint32_t n = us * 30U;  // ~120 MHz core, conservative
  while (n-- > 0U) {
    __NOP();
  }
}

static void tle_gpio_config(void) {
  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_AF);

  // Free PB3/PB4/PA15 from JTAG (keep SWD available for debugging).
  gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);

  // SCK + MOSI as AF push-pull, MISO floating input.
  gpio_init(TLE_SPI_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, TLE_SCK_PIN | TLE_MOSI_PIN);
  gpio_init(TLE_SPI_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, TLE_MISO_PIN);

  // CSQ managed manually; idle high.
  gpio_init(TLE_CS_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, TLE_CS_PIN);
  gpio_bit_set(TLE_CS_PORT, TLE_CS_PIN);
}

static void tle_spi_config(void) {
  spi_parameter_struct spi;

  rcu_periph_clock_enable(RCU_SPI2);
  spi_i2s_deinit(TLE_SPI);
  spi_struct_para_init(&spi);

  spi.device_mode = SPI_MASTER;
  spi.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
  spi.frame_size = SPI_FRAMESIZE_16BIT;
  spi.nss = SPI_NSS_SOFT;
  spi.endian = SPI_ENDIAN_MSB;
  spi.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;  // SPI mode 1 (output rising, sample falling)
  spi.prescale = SPI_PSC_16;                          // 60 MHz / 16 = 3.75 MHz
  spi_init(TLE_SPI, &spi);

  spi_nss_internal_high(TLE_SPI);
  spi_dma_enable(TLE_SPI, SPI_DMA_RECEIVE);
  spi_dma_enable(TLE_SPI, SPI_DMA_TRANSMIT);
  spi_enable(TLE_SPI);
}

static void tle_dma_config(void) {
  dma_parameter_struct dma;

  rcu_periph_clock_enable(RCU_DMA1);

  // RX: SPI data register -> rx_word (single data word).
  dma_deinit(TLE_DMA, TLE_DMA_RX_CH);
  dma_struct_para_init(&dma);
  dma.periph_addr = (uint32_t)&SPI_DATA(TLE_SPI);
  dma.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
  dma.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma.memory_addr = (uint32_t)&rx_word;
  dma.memory_width = DMA_MEMORY_WIDTH_16BIT;
  dma.memory_inc = DMA_MEMORY_INCREASE_DISABLE;
  dma.direction = DMA_PERIPHERAL_TO_MEMORY;
  dma.number = 1U;
  dma.priority = DMA_PRIORITY_ULTRA_HIGH;
  dma_init(TLE_DMA, TLE_DMA_RX_CH, &dma);
  dma_circulation_disable(TLE_DMA, TLE_DMA_RX_CH);
  dma_interrupt_enable(TLE_DMA, TLE_DMA_RX_CH, DMA_INT_FTF);

  // TX: tx_dummy -> SPI data register (clocks the data word out).
  dma_deinit(TLE_DMA, TLE_DMA_TX_CH);
  dma_struct_para_init(&dma);
  dma.periph_addr = (uint32_t)&SPI_DATA(TLE_SPI);
  dma.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
  dma.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma.memory_addr = (uint32_t)&tx_dummy;
  dma.memory_width = DMA_MEMORY_WIDTH_16BIT;
  dma.memory_inc = DMA_MEMORY_INCREASE_DISABLE;
  dma.direction = DMA_MEMORY_TO_PERIPHERAL;
  dma.number = 1U;
  dma.priority = DMA_PRIORITY_HIGH;
  dma_init(TLE_DMA, TLE_DMA_TX_CH, &dma);
  dma_circulation_disable(TLE_DMA, TLE_DMA_TX_CH);

  nvic_irq_enable(TLE_DMA_RX_IRQn, 2U, 0U);
}

// Blocking 16-bit transfer with the DMA channels off (command phase).
static uint16_t tle_spi_xfer16(uint16_t word) {
  while (spi_i2s_flag_get(TLE_SPI, SPI_FLAG_TBE) == RESET) {}
  spi_i2s_data_transmit(TLE_SPI, word);
  while (spi_i2s_flag_get(TLE_SPI, SPI_FLAG_RBNE) == RESET) {}
  return spi_i2s_data_receive(TLE_SPI);
}

bool tle5012b_init(void) {
  read_busy = false;
  angle_raw = 0U;

  tle_gpio_config();
  tle_spi_config();
  tle_dma_config();

  uint32_t guard = TLE_READ_TIMEOUT;
  tle5012b_start_read();
  while (tle5012b_busy() && (guard-- > 0U)) {}
  return guard > 0U;
}

void tle5012b_attach_callback(void (*cb)(void)) {
  s_done_callback = cb;
}

void tle5012b_start_read(void) {
  read_busy = true;

  gpio_bit_reset(TLE_CS_PORT, TLE_CS_PIN);
  tle_delay_us(1U);  // CS setup (t_CSs >= 105 ns)

  // Command phase: MOSI drives the read command. DMA channels are off here.
  PB5_AS_AF();
  dbg_cmd_echo = tle_spi_xfer16(TLE_CMD_READ_AVAL);

  // Release the shared DATA line so the sensor can drive it, then wait the
  // turnaround time (t_wr_delay >= 130 ns) before clocking the data word.
  PB5_AS_INPUT();
  tle_delay_us(2U);

  // Data phase: clock one dummy word; the sensor drives the angle onto DATA,
  // the SPI reads it on MISO (PB4) and DMA stores it.
  dma_channel_disable(TLE_DMA, TLE_DMA_RX_CH);
  dma_channel_disable(TLE_DMA, TLE_DMA_TX_CH);
  dma_flag_clear(TLE_DMA, TLE_DMA_RX_CH, DMA_FLAG_G);
  dma_flag_clear(TLE_DMA, TLE_DMA_TX_CH, DMA_FLAG_G);
  dma_transfer_number_config(TLE_DMA, TLE_DMA_RX_CH, 1U);
  dma_transfer_number_config(TLE_DMA, TLE_DMA_TX_CH, 1U);

  dma_channel_enable(TLE_DMA, TLE_DMA_RX_CH);
  dma_channel_enable(TLE_DMA, TLE_DMA_TX_CH);
}

bool tle5012b_busy(void) {
  return read_busy;
}

uint16_t tle5012b_get_raw(void) {
  return angle_raw;
}

float tle5012b_get_angle_rad(void) {
  return (float)angle_raw * (TLE_2PI / TLE_ANGLE_RES);
}

uint16_t tle5012b_read_raw_blocking(void) {
  uint32_t guard = TLE_READ_TIMEOUT;
  tle5012b_start_read();
  while (tle5012b_busy() && (guard-- > 0U)) {}
  return angle_raw;
}

// Diagnostic: MISO value clocked back during the command word (~0x8020 on a
// healthy bus). 0xFFFF/0x0000 means SCK/MOSI/MISO are not reaching the sensor.
uint16_t tle5012b_debug_cmd_echo(void) {
  return dbg_cmd_echo;
}

void DMA1_Channel0_IRQHandler(void) {
  if (dma_interrupt_flag_get(TLE_DMA, TLE_DMA_RX_CH, DMA_INT_FLAG_FTF) == SET) {
    dma_interrupt_flag_clear(TLE_DMA, TLE_DMA_RX_CH, DMA_INT_FLAG_FTF);

    while (spi_i2s_flag_get(TLE_SPI, SPI_FLAG_TRANS) != RESET) {}
    gpio_bit_set(TLE_CS_PORT, TLE_CS_PIN);
    PB5_AS_AF();  // restore MOSI drive for the next command

    dma_channel_disable(TLE_DMA, TLE_DMA_RX_CH);
    dma_channel_disable(TLE_DMA, TLE_DMA_TX_CH);

    angle_raw = rx_word & TLE_ANGLE_MASK;
    read_busy = false;

    if (s_done_callback != 0) {
      s_done_callback();
    }
  }
}
