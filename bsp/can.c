#include "can.h"

#include "gd32f30x.h"

// --- Hardware mapping: CAN0 RX=PA11, TX=PA12 (default pins, no remap) ---
#define CAN_PERIPH        CAN0
#define CAN_GPIO_PORT     GPIOA
#define CAN_RX_PIN        GPIO_PIN_11
#define CAN_TX_PIN        GPIO_PIN_12
#define CAN_BAUDRATE      1000000U

// Bit timing from APB1 = 60 MHz: baud = 60 MHz / prescaler / (1 + BS1 + BS2).
// 1 Mbps: prescaler 4, BS1 11 tq, BS2 3 tq -> 15 tq/bit, 80% sample point.
// 500 kbps alternative: prescaler 8 with the same tq segments.
// Named CAN_CFG_* (not CAN_BT_*): the SPL already uses the CAN_BT_ prefix for
// the CAN_BT register's bitfield MASKS (CAN_BT_BS1 = BITS(16,19), etc). Reusing
// those names redefined the SPL macros — harmless only because this file never
// touches the register masks. A distinct prefix removes the collision.
#define CAN_CFG_PRESCALER  4U
#define CAN_CFG_BS1        CAN_BT_BS1_11TQ
#define CAN_CFG_BS2        CAN_BT_BS2_3TQ
#define CAN_CFG_SJW        CAN_BT_SJW_1TQ

// 1 = internal loopback self-test (no transceiver/bus needed), 0 = normal bus.
#define CAN_SELFTEST_LOOPBACK  0U

// 1 = hardware automatic bus-off recovery (ABOM): after a bus-off the
// controller rejoins the bus on its own (128 x 11 recessive bits); 0 = stay
// off-bus until re-init.
#define CAN_AUTO_BUS_OFF_RECOVERY  1U

// RX ring depth in frames. Single producer (ISR) / single consumer (main
// loop); the head/tail indices are each written by one side only, so no
// locking is needed (16-bit accesses are atomic on Cortex-M).
#define CAN_RX_RING_SIZE  16U

// NVIC priorities. CAN is a comms path, so it stays below the FOC loop (same
// level as the UART).
#define CAN_IRQ_PRE_PRIO  3U
#define CAN_IRQ_SUB_PRIO  0U

static can_frame_t rx_ring[CAN_RX_RING_SIZE];
static volatile uint16_t rx_head;    // written by the RX ISR only
static volatile uint16_t rx_tail;    // written by the consumer only
static volatile uint16_t rx_dropped; // frames lost to a full ring

static void can_gpio_config(void);
static void can_periph_config(void);
static void can_filter_config(void);

void can_bus_init(void) {
  rx_head = 0U;
  rx_tail = 0U;
  rx_dropped = 0U;

  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(RCU_CAN0);

  can_gpio_config();
  can_periph_config();
  can_filter_config();

  can_interrupt_enable(CAN_PERIPH, CAN_INT_RFNE0);
  nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, CAN_IRQ_PRE_PRIO, CAN_IRQ_SUB_PRIO);
}

static void can_gpio_config(void) {
  gpio_init(CAN_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, CAN_RX_PIN);
  gpio_init(CAN_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, CAN_TX_PIN);
}

static void can_periph_config(void) {
  can_parameter_struct param;

  can_deinit(CAN_PERIPH);
  can_struct_para_init(CAN_INIT_STRUCT, &param);
#if CAN_SELFTEST_LOOPBACK
  param.working_mode = CAN_LOOPBACK_MODE;
#else
  param.working_mode = CAN_NORMAL_MODE;
#endif
  param.resync_jump_width     = CAN_CFG_SJW;
  param.time_segment_1        = CAN_CFG_BS1;
  param.time_segment_2        = CAN_CFG_BS2;
  param.prescaler             = CAN_CFG_PRESCALER;   // -> CAN_BAUDRATE, see above
  param.time_triggered        = DISABLE;
  param.auto_bus_off_recovery = CAN_AUTO_BUS_OFF_RECOVERY ? ENABLE : DISABLE;
  param.auto_wake_up          = DISABLE;
  param.no_auto_retrans       = DISABLE;  // hardware retransmits lost frames
  param.rec_fifo_overwrite    = DISABLE;  // keep the oldest on FIFO overrun
  param.trans_fifo_order      = DISABLE;  // transmit by identifier priority
  can_init(CAN_PERIPH, &param);
}

// Accept every frame into FIFO0 (mask 0 = don't care). can_bus_set_node_filter
// replaces this once the protocol layer knows which node it is.
static void can_filter_config(void) {
  can_filter_parameter_struct filter;

  can_struct_para_init(CAN_FILTER_STRUCT, &filter);
  filter.filter_number      = 0U;
  filter.filter_mode        = CAN_FILTERMODE_MASK;
  filter.filter_bits        = CAN_FILTERBITS_32BIT;
  filter.filter_list_high   = 0U;
  filter.filter_list_low    = 0U;
  filter.filter_mask_high   = 0U;
  filter.filter_mask_low    = 0U;
  filter.filter_fifo_number = CAN_FIFO0;
  filter.filter_enable      = ENABLE;
  can_filter_init(&filter);
}

// 32-bit filter layout (GD32F30x UM): SFID[10:0] sits at bits [15:5] of the
// high half-word; the low half-word's bit 2 is FF (0 = standard frame).
// Matching SFID[10:5] (mask 0xFC00) selects one node with the low 5 cmd bits
// don't-care; masking FF (0x0004, list 0) lets standard frames through only.
#define CAN_FILTER_NODE_MASK_HIGH  0xFC00U
#define CAN_FILTER_STD_MASK_LOW    0x0004U

void can_bus_set_node_filter(uint8_t node_id) {
  can_filter_parameter_struct filter;
  uint16_t node_high = (uint16_t)((uint16_t)(node_id & 0x3FU) << 10);

  can_struct_para_init(CAN_FILTER_STRUCT, &filter);
  filter.filter_mode        = CAN_FILTERMODE_MASK;
  filter.filter_bits        = CAN_FILTERBITS_32BIT;
  filter.filter_mask_high   = CAN_FILTER_NODE_MASK_HIGH;
  filter.filter_mask_low    = CAN_FILTER_STD_MASK_LOW;
  filter.filter_fifo_number = CAN_FIFO0;
  filter.filter_enable      = ENABLE;

  /* filter 0: this node (ID = node<<5 | cmd, cmd don't care) */
  filter.filter_number    = 0U;
  filter.filter_list_high = node_high;
  filter.filter_list_low  = 0U;
  can_filter_init(&filter);

  /* filter 1: broadcast node 0 */
  filter.filter_number    = 1U;
  filter.filter_list_high = 0U;
  filter.filter_list_low  = 0U;
  can_filter_init(&filter);
}

uint8_t can_bus_send(uint32_t id, uint8_t dlc, const uint8_t *data) {
  can_trasnmit_message_struct msg;
  uint8_t i;

  msg.tx_sfid = id & 0x7FFU;
  msg.tx_efid = 0U;
  msg.tx_ff   = (uint8_t)CAN_FF_STANDARD;
  msg.tx_ft   = (uint8_t)CAN_FT_DATA;
  msg.tx_dlen = (dlc > 8U) ? 8U : dlc;
  for (i = 0U; i < msg.tx_dlen; i++) {
    msg.tx_data[i] = data[i];
  }

  return (can_message_transmit(CAN_PERIPH, &msg) != CAN_NOMAILBOX) ? 1U : 0U;
}

// Lock-free single-producer/single-consumer: the ISR writes only rx_head and
// never advances it onto rx_tail (full frames are dropped instead), so the
// slot being copied here can never be written by the ISR, and the 16-bit
// index reads are atomic. No critical section needed.
uint8_t can_bus_recv(can_frame_t *frame) {
  if (rx_head == rx_tail) {
    return 0U;
  }
  *frame = rx_ring[rx_tail];
  rx_tail = (uint16_t)((rx_tail + 1U) % CAN_RX_RING_SIZE);
  return 1U;
}

void can_bus_get_stats(uint16_t *dropped, uint8_t *tec, uint8_t *rec) {
  uint32_t err = CAN_ERR(CAN_PERIPH);

  *tec     = (uint8_t)GET_ERR_TECNT(err);
  *rec     = (uint8_t)GET_ERR_RECNT(err);
  *dropped = rx_dropped;
}

// FIFO0 not-empty: drain every pending frame into the ring. When the ring is
// full the frame is still popped (the FIFO must be drained) but discarded and
// counted, so a stalled consumer degrades to drops instead of a stuck FIFO.
void USBD_LP_CAN0_RX0_IRQHandler(void) {
  can_receive_message_struct msg;
  can_frame_t *f;
  uint16_t next;
  uint8_t i;

  while (can_receive_message_length_get(CAN_PERIPH, CAN_FIFO0) > 0U) {
    can_message_receive(CAN_PERIPH, CAN_FIFO0, &msg);

    next = (uint16_t)((rx_head + 1U) % CAN_RX_RING_SIZE);
    if (next == rx_tail) {
      rx_dropped++;
      continue;
    }

    f = &rx_ring[rx_head];
    f->ide = (msg.rx_ff == CAN_FF_EXTENDED) ? 1U : 0U;
    f->id  = f->ide ? msg.rx_efid : msg.rx_sfid;
    f->rtr = (msg.rx_ft == CAN_FT_REMOTE) ? 1U : 0U;
    f->dlc = (msg.rx_dlen > 8U) ? 8U : msg.rx_dlen;
    for (i = 0U; i < f->dlc; i++) {
      f->data[i] = msg.rx_data[i];
    }
    rx_head = next;
  }
}
