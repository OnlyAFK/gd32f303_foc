#include "uart.h"

#include "gd32f30x.h"
#include "lwrb.h"

// --- Hardware mapping: USART2 TX=PB10, RX=PB11 (default pins, no remap) ---
#define UART_PERIPH        USART2
#define UART_GPIO_PORT     GPIOB
#define UART_TX_PIN        GPIO_PIN_10
#define UART_RX_PIN        GPIO_PIN_11
#define UART_BAUDRATE      3000000U

// USART2_TX -> DMA0 channel1, USART2_RX -> DMA0 channel2 (fixed mapping).
#define UART_DMA           DMA0
#define UART_DMA_TX_CH     DMA_CH1
#define UART_DMA_RX_CH     DMA_CH2

// Ring buffer sizes. lwrb keeps one slot reserved, so usable size is N-1.
#define UART_TX_RB_SIZE    1024U
#define UART_RX_RB_SIZE    1024U
// Hardware DMA landing buffer for RX. IDLE + half/full events drain it.
#define UART_RX_DMA_SIZE   256U

// NVIC priorities. UART is telemetry only, so it stays below the FOC loop.
#define UART_IRQ_PRE_PRIO  3U
#define UART_IRQ_SUB_PRIO  0U

static lwrb_t tx_rb;
static lwrb_t rx_rb;
static uint8_t tx_rb_data[UART_TX_RB_SIZE];
static uint8_t rx_rb_data[UART_RX_RB_SIZE];
static uint8_t rx_dma_buf[UART_RX_DMA_SIZE];

static volatile uint8_t tx_busy;     // a DMA TX transfer is in flight
static volatile uint32_t tx_dma_len; // bytes handed to the current TX transfer
static uint32_t rx_old_pos;          // last consumed offset in rx_dma_buf

static void uart_gpio_config(void);
static void uart_dma_config(void);
static void uart_tx_kick(void);
static void uart_rx_process(void);

void uart_init(void) {
  lwrb_init(&tx_rb, tx_rb_data, sizeof(tx_rb_data));
  lwrb_init(&rx_rb, rx_rb_data, sizeof(rx_rb_data));
  tx_busy = 0;
  tx_dma_len = 0;
  rx_old_pos = 0;

  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(RCU_USART2);
  rcu_periph_clock_enable(RCU_DMA0);

  uart_gpio_config();
  uart_dma_config();

  usart_deinit(UART_PERIPH);
  usart_baudrate_set(UART_PERIPH, UART_BAUDRATE);
  usart_word_length_set(UART_PERIPH, USART_WL_8BIT);
  usart_stop_bit_set(UART_PERIPH, USART_STB_1BIT);
  usart_parity_config(UART_PERIPH, USART_PM_NONE);
  usart_hardware_flow_rts_config(UART_PERIPH, USART_RTS_DISABLE);
  usart_hardware_flow_cts_config(UART_PERIPH, USART_CTS_DISABLE);
  usart_receive_config(UART_PERIPH, USART_RECEIVE_ENABLE);
  usart_transmit_config(UART_PERIPH, USART_TRANSMIT_ENABLE);
  usart_dma_receive_config(UART_PERIPH, USART_DENR_ENABLE);
  usart_dma_transmit_config(UART_PERIPH, USART_DENT_ENABLE);

  // RX is framed by the IDLE line event; drain whatever the DMA has landed.
  usart_interrupt_enable(UART_PERIPH, USART_INT_IDLE);

  nvic_irq_enable(USART2_IRQn, UART_IRQ_PRE_PRIO, UART_IRQ_SUB_PRIO);
  nvic_irq_enable(DMA0_Channel1_IRQn, UART_IRQ_PRE_PRIO, UART_IRQ_SUB_PRIO);
  nvic_irq_enable(DMA0_Channel2_IRQn, UART_IRQ_PRE_PRIO, UART_IRQ_SUB_PRIO);

  usart_enable(UART_PERIPH);

  // Start the circular RX DMA; it runs forever and wraps on its own.
  dma_channel_enable(UART_DMA, UART_DMA_RX_CH);
}

static void uart_gpio_config(void) {
  gpio_init(UART_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, UART_TX_PIN);
  gpio_init(UART_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, UART_RX_PIN);
}

static void uart_dma_config(void) {
  dma_parameter_struct dma;

  // RX: peripheral -> circular memory buffer, never stops.
  dma_deinit(UART_DMA, UART_DMA_RX_CH);
  dma_struct_para_init(&dma);
  dma.periph_addr = (uint32_t)&USART_DATA(UART_PERIPH);
  dma.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
  dma.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma.memory_addr = (uint32_t)rx_dma_buf;
  dma.memory_width = DMA_MEMORY_WIDTH_8BIT;
  dma.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma.direction = DMA_PERIPHERAL_TO_MEMORY;
  dma.number = UART_RX_DMA_SIZE;
  dma.priority = DMA_PRIORITY_HIGH;
  dma_init(UART_DMA, UART_DMA_RX_CH, &dma);
  dma_circulation_enable(UART_DMA, UART_DMA_RX_CH);
  dma_interrupt_enable(UART_DMA, UART_DMA_RX_CH, DMA_INT_HTF | DMA_INT_FTF);

  // TX: memory -> peripheral, one-shot per transfer. Address/count set per kick.
  dma_deinit(UART_DMA, UART_DMA_TX_CH);
  dma_struct_para_init(&dma);
  dma.periph_addr = (uint32_t)&USART_DATA(UART_PERIPH);
  dma.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
  dma.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma.memory_addr = (uint32_t)tx_rb_data;
  dma.memory_width = DMA_MEMORY_WIDTH_8BIT;
  dma.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma.direction = DMA_MEMORY_TO_PERIPHERAL;
  dma.number = 1U;
  dma.priority = DMA_PRIORITY_MEDIUM;
  dma_init(UART_DMA, UART_DMA_TX_CH, &dma);
  dma_circulation_disable(UART_DMA, UART_DMA_TX_CH);
  dma_interrupt_enable(UART_DMA, UART_DMA_TX_CH, DMA_INT_FTF);
}

uint32_t uart_write(const void *data, uint32_t len) {
  uint32_t written = lwrb_write(&tx_rb, data, len);
  uart_tx_kick();
  return written;
}

uint32_t uart_read(void *data, uint32_t len) {
  return lwrb_read(&rx_rb, data, len);
}

uint32_t uart_rx_available(void) {
  return lwrb_get_full(&rx_rb);
}

static const io_stream_t s_uart_stream = {
  .write = uart_write,
  .read = uart_read,
};

const io_stream_t *uart_stream(void) {
  return &s_uart_stream;
}

// Start a DMA transfer of the next contiguous chunk in the TX ring buffer.
// Safe to call from both thread and ISR context: PRIMASK is saved/restored so
// the busy check and channel start cannot be interrupted by the TX-done ISR.
static void uart_tx_kick(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (tx_busy) {
    __set_PRIMASK(primask);
    return;
  }

  uint32_t len = lwrb_get_linear_block_read_length(&tx_rb);
  if (len == 0U) {
    __set_PRIMASK(primask);
    return;
  }

  void *addr = lwrb_get_linear_block_read_address(&tx_rb);
  tx_dma_len = len;
  tx_busy = 1U;

  dma_channel_disable(UART_DMA, UART_DMA_TX_CH);
  dma_memory_address_config(UART_DMA, UART_DMA_TX_CH, (uint32_t)addr);
  dma_transfer_number_config(UART_DMA, UART_DMA_TX_CH, len);
  dma_channel_enable(UART_DMA, UART_DMA_TX_CH);

  __set_PRIMASK(primask);
}

// Move bytes the RX DMA has landed since last call into the RX ring buffer.
// Called from the IDLE and DMA half/full interrupts.
static void uart_rx_process(void) {
  uint32_t pos = UART_RX_DMA_SIZE - dma_transfer_number_get(UART_DMA, UART_DMA_RX_CH);

  if (pos == rx_old_pos) {
    return;
  }

  if (pos > rx_old_pos) {
    lwrb_write(&rx_rb, &rx_dma_buf[rx_old_pos], pos - rx_old_pos);
  } else {
    lwrb_write(&rx_rb, &rx_dma_buf[rx_old_pos], UART_RX_DMA_SIZE - rx_old_pos);
    if (pos > 0U) {
      lwrb_write(&rx_rb, &rx_dma_buf[0], pos);
    }
  }

  rx_old_pos = pos;
}

void USART2_IRQHandler(void) {
  if (usart_interrupt_flag_get(UART_PERIPH, USART_INT_FLAG_IDLE) == SET) {
    // IDLE is cleared by the read STAT0 -> read DATA software sequence.
    volatile uint32_t tmp = USART_STAT0(UART_PERIPH);
    tmp = USART_DATA(UART_PERIPH);
    (void)tmp;
    uart_rx_process();
  }
}

void DMA0_Channel1_IRQHandler(void) {
  if (dma_interrupt_flag_get(UART_DMA, UART_DMA_TX_CH, DMA_INT_FLAG_FTF) == SET) {
    dma_interrupt_flag_clear(UART_DMA, UART_DMA_TX_CH, DMA_INT_FLAG_FTF);
    dma_channel_disable(UART_DMA, UART_DMA_TX_CH);
    lwrb_skip(&tx_rb, tx_dma_len);
    tx_busy = 0U;
    uart_tx_kick();
  }
}

void DMA0_Channel2_IRQHandler(void) {
  if (dma_interrupt_flag_get(UART_DMA, UART_DMA_RX_CH, DMA_INT_FLAG_HTF) == SET) {
    dma_interrupt_flag_clear(UART_DMA, UART_DMA_RX_CH, DMA_INT_FLAG_HTF);
    uart_rx_process();
  }
  if (dma_interrupt_flag_get(UART_DMA, UART_DMA_RX_CH, DMA_INT_FLAG_FTF) == SET) {
    dma_interrupt_flag_clear(UART_DMA, UART_DMA_RX_CH, DMA_INT_FLAG_FTF);
    uart_rx_process();
  }
}
