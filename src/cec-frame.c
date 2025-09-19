#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "cec-frame.h"
#include "cec-log.h"
#include "cec-task.h"
#include "usb-cdc.h" // For cdc_printfln debug output

#define NOTIFY_RX ((UBaseType_t)0)
#define NOTIFY_TX ((UBaseType_t)1)

TaskHandle_t xCECTask;

static uint8_t rx_buffer[16] = {0x0};
static cec_message_t rx_message = {.data = &rx_buffer[0], .len = 0};
static cec_frame_t rx_frame = {.message = &rx_message};

/* CEC statistics. */
static cec_frame_stats_t cec_stats;

/**
 * Calculate next offset as time since boot.
 */
static uint64_t time_next(uint64_t start, uint64_t next) {
  return (next - (time_us_64() - start));
}

/**
 * Pull the CEC line high at the specified time.
 */
static int64_t ack_high(alarm_id_t alarm, void *user_data) {
  gpio_set_dir(CEC_PIN, GPIO_IN);

  return 0;
}

static void frame_rx_isr(uint gpio, uint32_t events) {
  uint64_t low_time = 0;
  gpio_acknowledge_irq(gpio, events);
  gpio_set_irq_enabled(CEC_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
  // printf("state = %d, byte = %d, bit = %d\n", rx_frame.state, rx_frame.byte, rx_frame.bit);
  switch (rx_frame.state) {
    case CEC_FRAME_STATE_START_LOW:
      rx_frame.start = time_us_64();
      rx_frame.state = CEC_FRAME_STATE_START_HIGH;
      gpio_set_irq_enabled(CEC_PIN, GPIO_IRQ_EDGE_RISE, true);
      return;
    case CEC_FRAME_STATE_START_HIGH:
      low_time = time_us_64() - rx_frame.start;
      if (low_time >= 3500 && low_time <= 3900) {
        rx_frame.first = true;
        rx_frame.byte = 0;
        rx_frame.bit = 0;
        rx_frame.state = CEC_FRAME_STATE_DATA_LOW;
        gpio_set_irq_enabled(CEC_PIN, GPIO_IRQ_EDGE_FALL, true);
      } else {
        rx_frame.state = CEC_FRAME_STATE_ABORT;
        xTaskNotifyIndexedFromISR(xCECTask, NOTIFY_RX, 0, eNoAction, NULL);
      }
      return;
    case CEC_FRAME_STATE_EOM_LOW:
      rx_frame.byte++;
      rx_frame.bit = 0;
    case CEC_FRAME_STATE_DATA_LOW: {
      uint64_t min_time = rx_frame.first ? 4300 : 2050;
      uint64_t max_time = rx_frame.first ? 4700 : 2750;
      uint64_t bit_time = time_us_64() - rx_frame.start;
      if (bit_time >= min_time && bit_time <= max_time) {
        rx_frame.start = time_us_64();
        if (rx_frame.state == CEC_FRAME_STATE_EOM_LOW) {
          rx_frame.state = CEC_FRAME_STATE_EOM_HIGH;
        } else {
          rx_frame.state = CEC_FRAME_STATE_DATA_HIGH;
        }
        rx_frame.first = false;
        gpio_set_irq_enabled(CEC_PIN, GPIO_IRQ_EDGE_RISE, true);
      } else {
        rx_frame.state = CEC_FRAME_STATE_ABORT;
        xTaskNotifyIndexedFromISR(xCECTask, NOTIFY_RX, 0, eNoAction, NULL);
      }
    }
      return;
    case CEC_FRAME_STATE_EOM_HIGH:
    case CEC_FRAME_STATE_DATA_HIGH:
      low_time = time_us_64() - rx_frame.start;
      uint8_t bit = false;
      if (low_time >= 400 && low_time <= 800) {
        bit = true;
      } else if (low_time >= 1300 && low_time <= 1700) {
        bit = false;
      } else {
        rx_frame.state = CEC_FRAME_STATE_ABORT;
        xTaskNotifyIndexedFromISR(xCECTask, NOTIFY_RX, 0, eNoAction, NULL);
        return;
      }
      if (rx_frame.state == CEC_FRAME_STATE_EOM_HIGH) {
        rx_frame.eom = bit;
        rx_frame.state = CEC_FRAME_STATE_ACK_LOW;
      } else {
        rx_frame.message->data[rx_frame.byte] <<= 1;
        rx_frame.message->data[rx_frame.byte] |= bit ? 0x01 : 0x00;
        rx_frame.bit++;
        if (rx_frame.bit > 7) {
          rx_frame.state = CEC_FRAME_STATE_EOM_LOW;
        } else {
          rx_frame.state = CEC_FRAME_STATE_DATA_LOW;
        }
      }
      gpio_set_irq_enabled(CEC_PIN, GPIO_IRQ_EDGE_FALL, true);
      return;
    case CEC_FRAME_STATE_ACK_LOW:
      rx_frame.start = time_us_64();
      // send ack by changing ack from 1 to 0
      uint8_t tgt_addr = rx_frame.message->data[0] & 0x0f;
      if ((tgt_addr != 0x0f) && (tgt_addr == rx_frame.address)) {
        rx_frame.state = CEC_FRAME_STATE_ACK_END;
        gpio_set_dir(CEC_PIN, GPIO_OUT);  // pull low, then schedule pull high
        add_alarm_at(from_us_since_boot(rx_frame.start + 1500), ack_high, NULL, true);
        rx_frame.ack = true;
      }
      rx_frame.state = CEC_FRAME_STATE_ACK_HIGH;
      gpio_set_irq_enabled(CEC_PIN, GPIO_IRQ_EDGE_RISE, true);
      return;
    case CEC_FRAME_STATE_ACK_HIGH:
      low_time = time_us_64() - rx_frame.start;
      if ((low_time >= 400 && low_time <= 800) || (low_time >= 1300 && low_time <= 1700)) {
        rx_frame.state = CEC_FRAME_STATE_ACK_END;
      } else {
        rx_frame.state = CEC_FRAME_STATE_ABORT;
        xTaskNotifyIndexedFromISR(xCECTask, NOTIFY_RX, 0, eNoAction, NULL);
        return;
      }
      // fall through
    case CEC_FRAME_STATE_ACK_END:
      if (rx_frame.eom) {
        rx_frame.state = CEC_FRAME_STATE_END;
      } else {
        rx_frame.state = CEC_FRAME_STATE_DATA_LOW;
        gpio_set_irq_enabled(CEC_PIN, GPIO_IRQ_EDGE_FALL, true);
        return;
      }
      // finish receiving frame
    case CEC_FRAME_STATE_END:
    default:
      rx_frame.message->len = rx_frame.byte;
      xTaskNotifyIndexedFromISR(xCECTask, NOTIFY_RX, 0, eNoAction, NULL);
  }
}

uint8_t cec_frame_recv(uint8_t *pld, uint8_t address) {
  // printf("cec_frame_recv\n");
  rx_frame.address = address;
  rx_frame.state = CEC_FRAME_STATE_START_LOW;
  rx_frame.ack = false;
  memset(&rx_frame.message->data[0], 0, 16);
  gpio_set_irq_enabled(CEC_PIN, GPIO_IRQ_EDGE_FALL, true);
  ulTaskNotifyTakeIndexed(NOTIFY_RX, pdTRUE, portMAX_DELAY);
  memcpy(pld, rx_frame.message->data, rx_frame.message->len);
  // printf("high water mark = %lu\n", uxTaskGetStackHighWaterMark(xCECTask));

  cec_log_frame(&rx_frame, true);

  // Global debug print for all received CEC frames
  if (cec_log_enabled()) {
    cdc_printfln("[CEC RX] len=%d: ", rx_frame.message->len);
    for (uint8_t i = 0; i < rx_frame.message->len; ++i) {
      cdc_printfln("  [%d]=0x%02x", i, rx_frame.message->data[i]);
    }
  }

  if (rx_frame.state == CEC_FRAME_STATE_ABORT) {
    // printf("ABORT\n");
    cec_stats.rx_abort_frames++;
    return 0;
  }

  cec_stats.rx_frames++;
  return rx_frame.message->len;
}

static int64_t frame_tx_callback(alarm_id_t alarm, void *user_data) {
  cec_frame_t *frame = (cec_frame_t *)user_data;
  uint64_t low_time = 0;

  // Debug output removed from timer callback to prevent deadlocks/freezes
  switch (frame->state) {
    case CEC_FRAME_STATE_START_LOW:
      gpio_set_dir(CEC_PIN, GPIO_OUT);
      frame->start = time_us_64();
      frame->state = CEC_FRAME_STATE_START_HIGH;
      return time_next(frame->start, 3700);
    case CEC_FRAME_STATE_START_HIGH:
      gpio_set_dir(CEC_PIN, GPIO_IN);
      frame->state = CEC_FRAME_STATE_DATA_LOW;
      return time_next(frame->start, 4500);
    case CEC_FRAME_STATE_DATA_LOW:
      gpio_set_dir(CEC_PIN, GPIO_OUT);
      frame->start = time_us_64();
      low_time = (frame->message->data[frame->byte] & (1 << frame->bit)) ? 600 : 1500;
      frame->state = CEC_FRAME_STATE_DATA_HIGH;
      return time_next(frame->start, low_time);
    case CEC_FRAME_STATE_DATA_HIGH:
      gpio_set_dir(CEC_PIN, GPIO_IN);
      if (frame->bit--) {
        frame->state = CEC_FRAME_STATE_DATA_LOW;
      } else {
        frame->byte++;
        frame->state = CEC_FRAME_STATE_EOM_LOW;
      }
      return time_next(frame->start, 2400);
    case CEC_FRAME_STATE_EOM_LOW:
      gpio_set_dir(CEC_PIN, GPIO_OUT);
      low_time = (frame->byte < frame->message->len) ? 1500 : 600;
      frame->start = time_us_64();
      frame->state = CEC_FRAME_STATE_EOM_HIGH;
      return time_next(frame->start, low_time);
    case CEC_FRAME_STATE_EOM_HIGH:
      gpio_set_dir(CEC_PIN, GPIO_IN);
      frame->state = CEC_FRAME_STATE_ACK_LOW;
      return time_next(frame->start, 2400);
    case CEC_FRAME_STATE_ACK_LOW:
      gpio_set_dir(CEC_PIN, GPIO_OUT);
      frame->start = time_us_64();
      frame->state = CEC_FRAME_STATE_ACK_HIGH;
      return time_next(frame->start, 600);
    case CEC_FRAME_STATE_ACK_HIGH:
      gpio_set_dir(CEC_PIN, GPIO_IN);
      if (frame->byte < frame->message->len) {
        frame->bit = 7;
        frame->state = CEC_FRAME_STATE_DATA_LOW;
        return time_next(frame->start, 2400);
      } else {
        frame->state = CEC_FRAME_STATE_ACK_WAIT;
        // middle of safe sample period (0.85ms, 1.25ms)
        return time_next(frame->start, (850 + 1250) / 2);
      }
    case CEC_FRAME_STATE_ACK_WAIT: {
      // Increased ACK sampling window and frequency
      // Window: 4ms, Frequency: every 5us (800 samples)
      // NOTE: Some TVs do not send an ACK pulse, or it may be too short to detect reliably in software.
      bool ack_detected = false;
      uint64_t ack_sample_start = time_us_64();
      uint64_t now = ack_sample_start;
      while ((now - ack_sample_start) < 4000) {
        int pin = gpio_get(CEC_PIN);
        if (!pin) {
          ack_detected = true;
          break;
        }
        uint64_t t0 = time_us_64();
        while ((time_us_64() - t0) < 5) { __asm volatile ("nop"); }
        now = time_us_64();
      }
      if (ack_detected) {
        frame->ack = true;
      }
      frame->state = CEC_FRAME_STATE_END;
      return time_next(frame->start, 2400);
    }
    case CEC_FRAME_STATE_END:
    default:
      xTaskNotifyIndexedFromISR(xCECTask, NOTIFY_TX, 0, eNoAction, NULL);
      return 0;
  }
}

static bool frame_tx(uint8_t *data, uint8_t len) {
  if (cec_log_enabled()) {
    cdc_printfln("[CEC DEBUG] frame_tx called! laddr=0x%02x, paddr=0x%04x", cec_get_logical_address(), cec_get_physical_address());
    cdc_printfln("[CEC DEBUG] Pin state before idle wait: %d", gpio_get(CEC_PIN));
    cdc_printfln("[CEC DEBUG] frame_tx called! If you see this, debug output is working.\n");
  }
  unsigned char i = 0;
  const int idle_timeout_ms = 500;
  absolute_time_t idle_start = get_absolute_time();
  if (cec_log_enabled()) {
    cdc_printfln("[CEC] Waiting for idle...\n");
    cdc_printfln("[CEC] Pin state before idle wait: %d\n", gpio_get(CEC_PIN));
  }
  // wait 7 bit times of idle before sending
  while (i < 7) {
    vTaskDelay(pdMS_TO_TICKS(2.4));
    int pin_state = gpio_get(CEC_PIN);
    if (cec_log_enabled()) {
      cdc_printfln("[CEC DEBUG] Pin state during idle wait: %d (i=%d)", pin_state, i);
    }
    if (pin_state) {
      i++;
    } else {
      // reset
      i = 0;
    }
    if (absolute_time_diff_us(idle_start, get_absolute_time()) > idle_timeout_ms * 1000) {
      if (cec_log_enabled()) {
        cdc_printfln("[CEC DEBUG] Idle wait timeout!");
      }
      return false;
    }
  }
  if (cec_log_enabled()) {
    cdc_printfln("[CEC DEBUG] Pin state after idle wait: %d", gpio_get(CEC_PIN));
  }

  cec_message_t message = {data, len};
  cec_frame_t frame = {.message = &message,
                       .bit = 7,
                       .byte = 0,
                       .start = 0,
                       .ack = false,
                       .state = CEC_FRAME_STATE_START_LOW};
  if (cec_log_enabled()) {                     
    cdc_printfln("[CEC] Starting TX...\n");
  }
  add_alarm_at(from_us_since_boot(time_us_64()), frame_tx_callback, &frame, true);
  // Add timeout to notification wait
  const TickType_t notify_timeout = pdMS_TO_TICKS(1000); // 1s
  BaseType_t notified = ulTaskNotifyTakeIndexed(NOTIFY_TX, pdTRUE, notify_timeout);
  if (cec_log_enabled()) {
    if (!notified) {
      cdc_printfln("[CEC] TX notify timeout!\n");
      return false;
    }
    cec_log_frame(&frame, false);
  }

  if (frame.ack) {
    cec_stats.tx_frames++;
  } else {
    cec_stats.tx_noack_frames++;
  }

  return frame.ack;
}

bool cec_frame_send(uint8_t pldcnt, uint8_t *pld) {
  if (cec_log_enabled()) {
    cdc_printfln("[CEC] cec_frame_send triggered\n");
  }
  // disable GPIO ISR for sending
  gpio_set_irq_enabled(CEC_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
  if (cec_log_enabled()) {
    cdc_printfln("[CEC] cec_frame_send: pldcnt=%d\n", pldcnt);
  }
  if (cec_log_enabled()) {
    for (int i = 0; i < pldcnt; ++i) {
      cdc_printfln("[CEC] pld[%d]=0x%02x\n", i, pld[i]);
    }
  }
  bool result = frame_tx(pld, pldcnt);
  if (cec_log_enabled()) {
    cdc_printfln("[CEC] cec_frame_send done, result=%d\n", result);
  }
  return result;
}

void cec_frame_get_stats(cec_frame_stats_t *stats) {
  *stats = cec_stats;
}

void cec_frame_init(void) {
  gpio_init(CEC_PIN);
  gpio_disable_pulls(CEC_PIN);
  gpio_set_dir(CEC_PIN, GPIO_IN);

  gpio_set_irq_callback(&frame_rx_isr);
  irq_set_enabled(IO_IRQ_BANK0, true);
  gpio_set_irq_enabled(CEC_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
}
