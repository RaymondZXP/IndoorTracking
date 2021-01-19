#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "product.h"
#include "timer.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"

#define GPIO_NUMBER_LED0 13
#define GPIO_NUMBER_LED1 14
#define RX_TIMEOUT 40 /** Time to wait for response  x16us */
#define TRX_PERIOD 45 /** Time from packet sent to Tx enable  x16us */

#define DATABASE 0x20001000 /** Base address for measurement database */
#define DATA_SIZE 128
#define NUM_BINS 128
#define NUM_SLAVES 4
#define NUMBER_OF_MEASUREMENTS NUM_BINS * 10

#define BLE2M

// UART definitions
#define MAX_TEST_DATA_BYTES (15U) /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256      /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256      /**< UART RX buffer size. */
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

static uint8_t master_id = 0x1;
static uint8_t test_frame[255] = { 0x00, 0x04, 0xFF, 0xC1, 0xFB, 0xE7 };  // 0-1 crc and verification, 2: master id, 3:
                                                                          // NONE 4: tag processing time, 5: slave_ID

static uint32_t tx_pkt_counter = 0;
static uint32_t radio_freqs[3] = { 0, 26, 78 };

static uint32_t timeout;
static uint32_t telp;
static uint32_t rx_pkt_counter = 0;
static uint32_t rx_pkt_counter_crcok = 0;
static uint32_t rx_timeouts = 0;
static uint32_t rx_ignored = 0;
static uint8_t rx_test_frame[256];

static uint32_t database[NUM_SLAVES][DATA_SIZE] __attribute__((section(".ARM.__at_DATABASE")));
// static uint32_t dbptr = 0;
static uint32_t bincnt[NUM_SLAVES][NUM_BINS];

static uint32_t highper = 0;
static uint32_t txcntw = 0;

static int radio_B_process_time = 4620;  // need to tune this

int channel_index = 1;
volatile bool freq_change = false;

void nrf_radio_init(void);

void nrf_radio_switch_channel(int radio_frequency_index);

void timer1_capture_init(uint32_t prescaler);

void TIMER1_IRQHandler(void);

void nrf_ppi_config(void);

void setup_leds();

void uart_error_handle(app_uart_evt_t* p_event);

void uart_init();

float calc_dist(void);

int main(void)
{
  nrf_ppi_config();
  setup_leds();
  uart_init();
  // printf("\r\nProgramme Started\r\n");

  uint32_t tempval, tempval1;
  int binNum;

  // int channel_index = 1;

  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  /* Puts zeros into bincnt */
  for (int i = 0; i < NUM_SLAVES; i++)
  {
    memset(bincnt[i], 0, sizeof bincnt[i]);
  }

  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
  {
  }

  /* Configure the timer with prescaler 0, counts every 1 cycle of timer clock (16MHz) */
  timer0_capture_init(0);
  timer1_capture_init(6);

  nrf_radio_init();
  NRF_CLOCK->TASKS_HFCLKSTART = 1; /* Start HFCLK */

  tx_pkt_counter = 0;

  while (true)
  {
    uint8_t cr;

    if (freq_change)
    {
      freq_change = !freq_change;
      channel_index = (channel_index + 1) % 3;
      nrf_radio_switch_channel(channel_index);
    }

    while (!freq_change)
    {
      NRF_RADIO->PACKETPTR = (uint32_t)test_frame; /* Switch to tx buffer */
      NRF_RADIO->TASKS_RXEN = 0x0;

      NRF_GPIO->OUTSET = 1 << GPIO_NUMBER_LED0; /* Rx LED Off */
      NRF_GPIO->OUTCLR = 1 << GPIO_NUMBER_LED1; /* Tx LED On */

      /* Copy the master id, tx packet counter into the payload and clear the slave_id*/
      // test_frame[2] = (tx_pkt_counter & 0x0000FF00) >> 8;
      test_frame[2] = master_id;
      test_frame[5] = 0x0;
      test_frame[4] = (tx_pkt_counter & 0x000000FF);

      NRF_TIMER0->TASKS_STOP = 1;
      NRF_TIMER0->TASKS_CLEAR = 1;

      /* Start Tx */
      NRF_RADIO->TASKS_TXEN = 0x1;

      /* Wait for transmision to begin */
      while (NRF_RADIO->EVENTS_READY == 0)
      {
      }

      NRF_RADIO->EVENTS_READY = 0;

      /* Packet is sent */
      while (NRF_RADIO->EVENTS_END == 0)
      {
      }
      NRF_RADIO->EVENTS_END = 0;

      /* Disable radio */
      while (NRF_RADIO->EVENTS_DISABLED == 0)
      {
      }
      NRF_RADIO->EVENTS_DISABLED = 0;

      tx_pkt_counter++;
      txcntw++;

      if (txcntw > 50)
      {
        txcntw = 0;
        if (rx_timeouts > 10)
          highper = 1;
        else
          highper = 0;

        rx_timeouts = 0;
      }

      /**
       * Packet sent, switch to Rx asap
       * Note: there is a small delay inserted on the B side to avoid race here
       */
      NRF_GPIO->OUTSET = 1 << GPIO_NUMBER_LED1; /* Tx LED Off */
      NRF_RADIO->TASKS_TXEN = 0x0;
      NRF_RADIO->TASKS_RXEN = 0x1;

      NRF_RADIO->PACKETPTR = (uint32_t)rx_test_frame; /* Switch to rx buffer*/

      /* Wait for response or timeout */
      timeout = 0;
      while ((NRF_RADIO->EVENTS_DISABLED == 0) && (timeout < 2048))
      {
        timeout++;
      }

      /* Now, did we time out? */
      if (timeout >= 2048)
      {
        /* Timeout, stop radio manually */
        NRF_RADIO->TASKS_STOP = 1;
        NRF_RADIO->TASKS_DISABLE = 1;
        while (NRF_RADIO->EVENTS_DISABLED == 0)
          ;
        rx_timeouts++;
        // break;
      }
      else
      {
        /* Packet received */
        if (highper < 1)
          NRF_GPIO->OUTCLR = 1 << GPIO_NUMBER_LED0; /* Rx LED On */

        rx_pkt_counter++;

        if (NRF_RADIO->CRCSTATUS > 0)
        {
          /**
           * Process the received packet
           * Check the sequence number in the received packet against our tx packet counter
           */
          rx_pkt_counter_crcok++;

          uint8_t rx_master_id = rx_test_frame[2];
          tempval = rx_test_frame[4];
          // uint8_t slave_id = rx_test_frame[5];

          tempval1 = (tx_pkt_counter - 1);

          if (tempval != (tempval1 & 0x000000FF) || rx_master_id != master_id)
            rx_ignored++;
          else
          {
            /* Packet is good, update stats */
            NRF_TIMER0->TASKS_STOP = 1;
            telp = NRF_TIMER0->CC[0];

            binNum = telp - radio_B_process_time;

            if ((binNum >= 0) && (binNum < NUM_BINS))
            {
              cr = binNum;
              app_uart_put(cr);
              // bincnt[slave_id][binNum]++;
            }

            NRF_TIMER0->TASKS_CLEAR = 1;
          }
        }
      }
      NRF_RADIO->EVENTS_DISABLED = 0;
    }

    // for (int j = 0; j < NUM_SLAVES; j++)
    // {
    //   for (int k = 0; k < NUM_BINS; k++)
    //   {
    //     database[j][k] = bincnt[j][k];
    //     bincnt[j][k] = 0;
    //   }
    // }

    // dbptr = 0;
    calc_dist();
  }
  NRF_GPIO->OUTSET = 1 << GPIO_NUMBER_LED1;
}

void nrf_radio_init(void)
{
  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                      (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

  NRF_RADIO->TIFS = 210;

#if defined(BLE2M)
  NRF_RADIO->MODE = 4 << RADIO_MODE_MODE_Pos;

  uint32_t aa_address = 0x71764129;
  NRF_RADIO->BASE0 = aa_address << 8;
  NRF_RADIO->PREFIX0 = (0xffffff00 | aa_address >> 24);

  NRF_RADIO->TXADDRESS = 0;
  NRF_RADIO->RXADDRESSES = 1;
  NRF_RADIO->DATAWHITEIV = 39;
  NRF_RADIO->PCNF0 = 0x01000108;

  NRF_RADIO->PCNF1 = 0x000300FF; /* sw:turn off whitening */
  NRF_RADIO->CRCPOLY = 0x65B;
  NRF_RADIO->CRCINIT = 0x555555;
  NRF_RADIO->CRCCNF = 0x103;
#endif

  NRF_RADIO->FREQUENCY =
      (RADIO_FREQUENCY_MAP_Default << RADIO_FREQUENCY_MAP_Pos) +  // 0*(2*8)
      ((radio_freqs[channel_index] << RADIO_FREQUENCY_FREQUENCY_Pos) & RADIO_FREQUENCY_FREQUENCY_Msk);

  NRF_RADIO->PACKETPTR = (uint32_t)test_frame;
  NRF_RADIO->EVENTS_DISABLED = 0;

  NRF_RADIO->TXPOWER = 0x0;
}

void nrf_radio_switch_channel(int radio_frequency_index)
{
  NRF_RADIO->TASKS_STOP = 1;
  NRF_RADIO->TASKS_DISABLE = 1;
  NRF_RADIO->FREQUENCY =
      (RADIO_FREQUENCY_MAP_Default << RADIO_FREQUENCY_MAP_Pos) +  // 0*(2*8)
      ((radio_freqs[radio_frequency_index] << RADIO_FREQUENCY_FREQUENCY_Pos) & RADIO_FREQUENCY_FREQUENCY_Msk);
}

void timer1_capture_init(uint32_t prescaler)
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

void nrf_ppi_config(void)
{
  NRF_PPI->CH[6].TEP = (uint32_t)(&NRF_TIMER0->TASKS_CAPTURE[0]);
  NRF_PPI->CH[6].EEP = (uint32_t)(&NRF_RADIO->EVENTS_ADDRESS);

  NRF_PPI->CH[7].TEP = (uint32_t)(&NRF_TIMER0->TASKS_START);
  NRF_PPI->CH[7].EEP = (uint32_t)(&NRF_RADIO->EVENTS_ADDRESS);

  NRF_PPI->CHENSET = (1 << 6) | (1 << 7);
}

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

void uart_error_handle(app_uart_evt_t* p_event);

void uart_init()
{
  uint32_t err_code;
  const app_uart_comm_params_t comm_params = { RX_PIN_NUMBER, TX_PIN_NUMBER, RTS_PIN_NUMBER,          CTS_PIN_NUMBER,
                                               UART_HWFC,     false,         NRF_UART_BAUDRATE_115200

  };
  APP_UART_FIFO_INIT(&comm_params, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, uart_error_handle, APP_IRQ_PRIORITY_LOWEST,
                     err_code);

  APP_ERROR_CHECK(err_code);
}

float calc_dist(void)
{
  float val = 0;
  int sum = 0;
  for (int s = 0; s < NUM_SLAVES; s++)
  {
    for (int i = 0; i < NUM_BINS; i++)
    {
      val += database[s][i] * (i + 1);  // number of measurements* traval time measured
      sum += database[s][i];            // number of measurements
    }

    if (val != 0)
    {
      val = val / sum;
      printf("\r\nDistance from slave %d: %d cm, Number of measurement:%d\r\n ", s, (int)(val * 100), sum);
    }

    val = 0;
    sum = 0;
  }

  return val;
}

void uart_error_handle(app_uart_evt_t* p_event)
{
  if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
  {
    APP_ERROR_HANDLER(p_event->data.error_communication);
  }
  else if (p_event->evt_type == APP_UART_FIFO_ERROR)
  {
    APP_ERROR_HANDLER(p_event->data.error_code);
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
