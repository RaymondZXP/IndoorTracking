// #include "nrf_delay.h"
// #include "boards.h"
#include "nrf52.h"
#include "nrf52_bitfields.h"

#define BLE2M


static uint32_t radio_freq = 26;
static uint32_t radio_freqs[3] = { 0, 26, 78 };
static uint8_t test_frame[256];
static uint32_t rx_pkt_counter = 0;
static uint32_t rx_pkt_counter_crcok = 0;
static uint32_t dbgcnt1 = 0;
static uint32_t tx_pkt_counter = 0;
static uint8_t response_test_frame[255] = { 0x00, 0x04, 0xFF, 0xC1, 0xFB, 0x03 };

volatile int freq_change = 0;
int channel_index = 0;

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

  nrf_radio_init();

  /* Setting radio in receive mode */
  NRF_RADIO->TASKS_RXEN = 0x1;
  NRF_RADIO->EVENTS_DISABLED = 0;

  while (1)
  {
    if (freq_change)
    {
      freq_change = !freq_change;
      channel_index = (channel_index + 1) % 3;
      nrf_radio_switch_channel(channel_index);
    }
    while (!freq_change)
    {


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
  while (1)
  {
  }
}

void MemoryManagement_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}
