#pragma once

#include "opendbc/safety/declarations.h"

// GCOV_EXCL_START
// Unreachable by design (doesn't define any rx msgs)
void default_rx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
}
// GCOV_EXCL_STOP

// *** no output safety mode ***

// Param 1 is a tightly-scoped offroad Tesla ambient-light test. It preserves
// no-output forwarding behavior and permits only one fixed-red accessory frame.
static bool nooutput_ambient_enabled = false;
static bool nooutput_ambient_template_valid = false;
static bool nooutput_ambient_tx_valid = false;
static uint8_t nooutput_ambient_template[7] = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
static uint32_t nooutput_ambient_template_ts = 0U;
static uint32_t nooutput_ambient_tx_ts = 0U;
static uint32_t nooutput_ambient_session_ts = 0U;
static uint8_t nooutput_ambient_session_count = 0U;

static const CanMsg NOOUTPUT_AMBIENT_TX_MSGS[] = {
  {0x679, 1, 7, .check_relay = false, .disable_static_blocking = true},
};

static void nooutput_ambient_reset(void) {
  nooutput_ambient_template_valid = false;
  nooutput_ambient_tx_valid = false;
  nooutput_ambient_template_ts = 0U;
  nooutput_ambient_tx_ts = 0U;
  nooutput_ambient_session_ts = 0U;
  nooutput_ambient_session_count = 0U;
  for (uint8_t i = 0U; i < 7U; i++) {
    nooutput_ambient_template[i] = 0U;
  }
}

static safety_config nooutput_init(uint16_t param) {
  nooutput_ambient_enabled = param == 1U;
  nooutput_ambient_reset();
  safety_config ret = (safety_config){NULL, 0, NULL, 0, true}; // NOLINT(readability/braces)
  if (nooutput_ambient_enabled) {
    ret.tx_msgs = NOOUTPUT_AMBIENT_TX_MSGS;
    ret.tx_msgs_len = sizeof(NOOUTPUT_AMBIENT_TX_MSGS) / sizeof(NOOUTPUT_AMBIENT_TX_MSGS[0]);
  }
  return ret;
}

static void nooutput_rx_observer(const CANPacket_t *msg) {
  if (nooutput_ambient_enabled && !msg->extended && !msg->returned && !msg->rejected && (msg->bus == 1U)) {
    if ((msg->addr == 0x679U) && (GET_LEN(msg) == 7U)) {
      for (uint8_t i = 0U; i < 7U; i++) {
        nooutput_ambient_template[i] = msg->data[i];
      }
      nooutput_ambient_template_ts = microsecond_timer_get();
      nooutput_ambient_template_valid = true;
    }
  }
}

static bool nooutput_tx_hook(const CANPacket_t *msg) {
  bool tx = false;
  if (nooutput_ambient_enabled && (msg->addr == 0x679U)) {
    const uint32_t now = microsecond_timer_get();
    const bool fresh = nooutput_ambient_template_valid &&
      (safety_get_ts_elapsed(now, nooutput_ambient_template_ts) <= 1000000U);
    const bool target = (((msg->data[5] & 0xF8U) == 0xA8U) && ((msg->data[6] & 1U) == 0U)) ||
                        (((msg->data[5] & 0xF8U) == 0x50U) && ((msg->data[6] & 1U) == 1U));
    const bool fixed_red = (msg->data[0] == ((nooutput_ambient_template[0] & 1U) | 2U)) &&
      (msg->data[1] == 255U) && (msg->data[2] == 0U) && (msg->data[3] == 0U) &&
      (msg->data[4] == ((nooutput_ambient_template[4] & 0x80U) | 100U)) &&
      ((msg->data[5] & 7U) == (nooutput_ambient_template[5] & 6U)) &&
      ((msg->data[6] & 0xFEU) == (nooutput_ambient_template[6] & 0xFEU));
    const bool new_session = !nooutput_ambient_tx_valid || (safety_get_ts_elapsed(now, nooutput_ambient_tx_ts) >= 1000000U);
    const bool rate = !nooutput_ambient_tx_valid || (safety_get_ts_elapsed(now, nooutput_ambient_tx_ts) >= 80000U);
    const bool session = new_session || ((safety_get_ts_elapsed(now, nooutput_ambient_session_ts) < 3000000U) &&
                                        (nooutput_ambient_session_count < 30U));
    tx = fresh && target && fixed_red && rate && session;
    if (tx) {
      if (new_session) {
        nooutput_ambient_session_ts = now;
        nooutput_ambient_session_count = 0U;
      }
      nooutput_ambient_session_count++;
      nooutput_ambient_tx_valid = true;
      nooutput_ambient_tx_ts = now;
    }
  }
  return tx;
}

const safety_hooks nooutput_hooks = {
  .init = nooutput_init,
  .rx = default_rx_hook,
  .rx_observer = nooutput_rx_observer,
  .tx = nooutput_tx_hook,
};

// *** all output safety mode ***
static safety_config alloutput_init(uint16_t param) {
  // Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa
  const uint16_t ALLOUTPUT_PARAM_PASSTHROUGH = 1;
  controls_allowed = true;
  bool alloutput_passthrough = GET_FLAG(param, ALLOUTPUT_PARAM_PASSTHROUGH);
  return (safety_config){NULL, 0, NULL, 0, !alloutput_passthrough}; // NOLINT(readability/braces)
}

static bool alloutput_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return true;
}

const safety_hooks alloutput_hooks = {
  .init = alloutput_init,
  .rx = default_rx_hook,
  .tx = alloutput_tx_hook,
};
