/*
 * telemetry.c
 *
 *  Created on: Dec 3, 2018
 *      Author: Robin
 */

#include "telemetry.h"

const uint8_t msg_marker[2] = {0xAB, 0xBA};

#define TELEMETRY_MAX_MSG_SIZE  18

static unsigned int msg_buffer_index = 0;
static unsigned int msg_in_marker = 0;
static unsigned int msg_in_header = 0;
static unsigned int msg_in_body = 0;
static unsigned int msg_expected_size = 0;
static unsigned int msg_error_cnt = 0;
static unsigned int msg_rx_cnt = 0;
static unsigned int msg_tx_cnt = 0;

static uint8_t msg_buffer_rx[TELEMETRY_MAX_MSG_SIZE];
static uint8_t msg_buffer_tx[TELEMETRY_MAX_MSG_SIZE];

typedef struct msg_callback {
  uint8_t msgid;
  void (*callback)(telemetry_msg_head_t *msg);
} msg_callback_t;

msg_callback_t msg_callbacks[10] = { 0 };

void telemetry_init(void)
{

}

void telemetry_reset(void)
{

}

void telemetry_process_byte(uint8_t ch)
{
  if (msg_in_body == 1) {
    msg_buffer_rx[msg_buffer_index++] = ch;
    if (msg_buffer_index == msg_expected_size) {
      telemetry_msg_head_t *head = (telemetry_msg_head_t *)&msg_buffer_rx[0];
      switch (head->id) {
      case 2:
      {
        telemetry_msg_set_speed_t *speed = (telemetry_msg_set_speed_t *)&msg_buffer_rx[0];
        telemetry_msg_set_speed_callback(speed);
        break;
      }
      case 3:
      {
        telemetry_msg_stop_t *stop = (telemetry_msg_stop_t *)&msg_buffer_rx[0];
        telemetry_msg_stop_callback(stop);
        break;
      }
      case 4:
      {
        telemetry_msg_start_t *start = (telemetry_msg_start_t *)&msg_buffer_rx[0];
        telemetry_msg_start_callback(start);
        break;
      }
      default:
      {
        break;
      }
      }

      msg_rx_cnt++;
      msg_in_body = 0;
      msg_buffer_index = 0;
    }
    else if (msg_buffer_index == TELEMETRY_MAX_MSG_SIZE) {
      msg_error_cnt++;
      msg_in_body = 0;
      msg_buffer_index = 0;
    }
  }
  else if (msg_in_header == 1) {
    if (msg_buffer_index == TELEMETRY_MAX_MSG_SIZE) {
      msg_error_cnt++;
      msg_in_header = 0;
      msg_buffer_index = 0;
    }

    msg_buffer_rx[msg_buffer_index++] = ch;

    if (msg_buffer_index == sizeof(telemetry_msg_head_t)) {
      telemetry_msg_head_t *head = (telemetry_msg_head_t *)&msg_buffer_rx[0];

      if (head->size > TELEMETRY_MAX_MSG_SIZE) {
        msg_error_cnt++;
        msg_in_header = 0;
        msg_buffer_index = 0;
      }
      else {
        msg_expected_size = head->size;
        msg_in_header = 0;
        msg_in_body = 1;
      }
    }
  }
  else {
    if (msg_in_marker == 0) {
      if (ch == msg_marker[0]) {
        msg_in_marker = 1;
      }
    }
    else {
      if (ch == msg_marker[1]) {
        msg_in_marker = 0;
        msg_in_header = 1;
      }
      else {
        msg_in_marker = 0;
      }
    }
  }
}

void telemetry_msg_status_pack(telemetry_msg_status_t *status, uint8_t mode, uint16_t vbus, uint16_t temp, uint16_t current, uint16_t speed)
{
  status->head.id = 1;
  status->head.size = sizeof(telemetry_msg_status_t);
  status->head.seq = 0;
  status->mode = mode;
  status->vbus = vbus;
  status->temp = temp;
  status->current = current;
  status->speed = speed;
}

void telemetry_msg_status_send(uint8_t mode, uint16_t vbus, uint16_t temp, uint16_t current, uint16_t speed)
{
  telemetry_msg_status_t status;
  telemetry_msg_status_pack(&status, mode, vbus, temp, current, speed);
  telemetry_msg_send(&status.head);
}

telemetry_msg_send(telemetry_msg_head_t *msg)
{
  telemetry_send_bytes((uint8_t *)msg, msg->size);
  msg_tx_cnt++;
}

__weak void telemetry_msg_set_speed_callback(telemetry_msg_set_speed_t *set_speed)
{
  (void)set_speed;
}

__weak void telemetry_msg_stop_callback(telemetry_msg_stop_t *stop)
{
  (void)stop;
}

__weak void telemetry_msg_start_callback(telemetry_msg_start_t *start)
{
  (void)start;
}

__weak void telemetry_send_byte(uint8_t ch)
{
  (void)ch;
}

__weak void telemetry_send_bytes(uint8_t *bytes, size_t len)
{
  (void)bytes;
  (void)len;
}
