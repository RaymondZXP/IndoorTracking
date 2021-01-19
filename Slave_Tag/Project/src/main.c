#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "timer.h"

#define DBG_PIN0 (28)
#define DBG_PIN1 (29)
#define DBG_PIN2 (30)
#define DBG_PIN3 (31)

#define NRF_GPIO NRF_P0

#define GPIO_NUMBER_LED0 13 /** LED for packet received */
#define GPIO_NUMBER_LED1 14 /** LED for packet sent */
#define TRXWAIT 4           /** Inserted turnaround latency x 16us */

#define BLE2M

static uint32_t radio_freq = 26;
static uint32_t radio_freqs[3] = { 0, 26, 78 };
static uint8_t test_frame[256];
static uint32_t rx_pkt_counter = 0;
static uint32_t rx_pkt_counter_crcok = 0;
static uint32_t dbgcnt1 = 0;
static uint32_t tx_pkt_counter = 0;

static uint8_t response_test_frame[255] = { 0x00, 0x04, 0xFF, 0xC1, 0xFB, 0x03 };

volatile bool freq_change = false;
int channel_index = 0;

/**
 * @brief Initializing the radio
 */
void nrf_radio_init(void)
{
#if defined(BLE2M)
  NRF_RADIO->MODE = 4 << RADIO_MODE_MODE_Pos; /* Radio in BLe 1M */
  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                      (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos) |
                      (RADIO_SHORTS_DISABLED_TXEN_Enabled << RADIO_SHORTS_DISABLED_TXEN_Pos);
  NRF_RADIO->PCNF0 = 0x01000108;
  NRF_RADIO->PCNF1 = 0x000300FF;
  NRF_RADIO->CRCPOLY = 0x65B;
  NRF_RADIO->CRCINIT = 0x555555;
  NRF_RADIO->CRCCNF = 0x103;
  NRF_RADIO->FREQUENCY = (RADIO_FREQUENCY_MAP_Default << RADIO_FREQUENCY_MAP_Pos) +
                         ((radio_freq << RADIO_FREQUENCY_FREQUENCY_Pos) & RADIO_FREQUENCY_FREQUENCY_Msk);
  NRF_RADIO->PACKETPTR = (uint32_t)test_frame;
  NRF_RADIO->EVENTS_READY = 0x0;
  NRF_RADIO->EVENTS_END = 0x0;

  uint32_t aa_address = 0x71764129;
  NRF_RADIO->BASE0 = aa_address << 8;
  NRF_RADIO->PREFIX0 = (0xffffff00 | aa_address >> 24);
  NRF_RADIO->TXADDRESS = 0;
  NRF_RADIO->RXADDRESSES = 1;
#endif

  NRF_RADIO->MODECNF0 = NRF_RADIO->MODECNF0 | 0x1F1F0000;
  NRF_RADIO->TIFS = 0x000000C0;
  NRF_RADIO->TXPOWER = 0x0;
}

void nrf_radio_switch_channel(int radio_frequency_index)
{
  NRF_RADIO->TASKS_STOP = 1;
  NRF_RADIO->TASKS_DISABLE = 1;
  NRF_RADIO->FREQUENCY =
      (RADIO_FREQUENCY_MAP_Default << RADIO_FREQUENCY_MAP_Pos) +  // 0*(2*8)
      ((radio_freqs[radio_frequency_index] << RADIO_FREQUENCY_FREQUENCY_Pos) & RADIO_FREQUENCY_FREQUENCY_Msk);
  // NRF_RADIO->TASKS_DISABLE = 0;
}
/**
 * @brief Setting up LEDs used for inication packet received or sent
 */
void setup_leds()
{
  NRF_GPIO->PIN_CNF[GPIO_NUMBER_LED0] =
      ((GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
       (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));

  NRF_GPIO->PIN_CNF[GPIO_NUMBER_LED1] =
      ((GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
       (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
}

void timer1_init(uint32_t prescaler)
{
  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Counter Mode
  NRF_TIMER1->TASKS_CLEAR = 1;               // clear the task first to be usable for later
  NRF_TIMER1->PRESCALER = prescaler;  // Set prescaler. Higher number gives slower timer. Prescaler = 0 gives 16MHz
                                      // timer
  NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit;  // Set counter to 16 bit resolution
  NRF_TIMER1->CC[0] = 10000;                          // Set value for TIMER1 compare register 0

  // Enable interrupt on Timer 1 for CC[0] compare match events
  NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
  NVIC_EnableIRQ(TIMER1_IRQn);

  NRF_TIMER1->TASKS_START = 1;  // Start TIMER1
}

void TIMER1_IRQHandler(void)
{
  if ((NRF_TIMER1->EVENTS_COMPARE[0] != 0) && ((NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
  {
    NRF_TIMER1->EVENTS_COMPARE[0] = 0;  // Clear compare register 0 event
    freq_change = true;
  }
}
int main(void)
{
  volatile uint32_t i;

  /* Start HFXO */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
  {
  }

  /* Configure the timer with prescaler 8,  counts every 256 cycle of timer clock (16MHz) */

  timer1_init(8);

  nrf_radio_init();
  setup_leds();

  /* Setting radio in receive mode */
  NRF_RADIO->TASKS_RXEN = 0x1;
  NRF_RADIO->EVENTS_DISABLED = 0;

  while (true)
  {
    if (freq_change)
    {
      freq_change = !freq_change;
      channel_index = (channel_index + 1) % 3;
      nrf_radio_switch_channel(channel_index);
    }
    while (!freq_change)
    {
      NRF_GPIO->OUTCLR = 1 << GPIO_NUMBER_LED0; /* Rx LED On */
      NRF_GPIO->OUTSET = 1 << GPIO_NUMBER_LED1; /* Tx LED Off */

      /* Wait for packet */
      while (NRF_RADIO->EVENTS_DISABLED == 0)
      {
      }
      NRF_RADIO->EVENTS_DISABLED = 0;

      /* Packet received, check CRC */
      rx_pkt_counter++;
      if (NRF_RADIO->CRCSTATUS > 0)
      {
        /* CRC ok */
        rx_pkt_counter_crcok++;

        for (i = 2; i < 5; i++)
          response_test_frame[i] = test_frame[i];
      }
      else
      {
        /* CRC error */
        dbgcnt1++;

        /* Insert zeros as sequence number into the response packet indicating crc error to initiator */
        for (i = 2; i < 5; i++)
          response_test_frame[i] = 0;
      }

      /* Switch to Tx asap and send response packet back to initiator */
      NRF_RADIO->PACKETPTR = (uint32_t)response_test_frame; /* Switch to tx buffer */
      NRF_GPIO->OUTSET = 1 << GPIO_NUMBER_LED0;             /* Rx LED Off */
      NRF_GPIO->OUTCLR = 1 << GPIO_NUMBER_LED1;             /* Tx LED On */
      NRF_RADIO->TASKS_RXEN = 0x0;
      // NRF_RADIO->TASKS_TXEN = 0x1;  /* Going via shortcut here */

      /* Wait for radio to be ready to transmit */
      while (NRF_RADIO->EVENTS_READY == 0)
      {
      }
      NRF_RADIO->EVENTS_READY = 0;

      /* Remove short DISABLED->TXEN before Tx ends */
      NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                          (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

      /* Wait for packet to be transmitted */
      while (NRF_RADIO->EVENTS_END == 0)
      {
      }
      NRF_RADIO->EVENTS_END = 0;

      /* Packet sent, disable radio */
      while (NRF_RADIO->EVENTS_DISABLED == 0)
      {
      }
      NRF_RADIO->EVENTS_DISABLED = 0;

      tx_pkt_counter++;

      /* Switch to Rx asap */
      NRF_RADIO->PACKETPTR = (uint32_t)test_frame; /* Switch to rx buffer */
      NRF_GPIO->OUTSET = 1 << GPIO_NUMBER_LED1;    /* Tx LED Off */
      NRF_RADIO->TASKS_TXEN = 0x0;
      NRF_RADIO->TASKS_RXEN = 0x1;

      /* Enable radio */
      while (NRF_RADIO->EVENTS_READY == 0)
      {
      }
      NRF_RADIO->EVENTS_READY = 0;

      /* Enable short DISABLED->TXEN before Rx goes to DISABLED */
      NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                          (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos) |
                          (RADIO_SHORTS_DISABLED_TXEN_Enabled << RADIO_SHORTS_DISABLED_TXEN_Pos);
    }
  }
}

void HardFault_Handler(void)
{
  while (true)
  {
  }
}

void MemoryManagement_Handler(void)
{
  while (true)
  {
  }
}

void BusFault_Handler(void)
{
  while (true)
  {
  }
}

void UsageFault_Handler(void)
{
  while (true)
  {
  }
}
