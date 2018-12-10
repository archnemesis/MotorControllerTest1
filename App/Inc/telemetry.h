/*
 * telemetry.h
 *
 *  Created on: Dec 3, 2018
 *      Author: Robin
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include <stddef.h>
#include <stdint.h>

#ifndef PACKED
#define PACKED __attribute__((packed))
#endif

typedef struct telemetry_msg_head {
  uint8_t id;
  uint32_t seq;
  uint16_t size;
} PACKED telemetry_msg_head_t;

typedef struct telemetry_msg_status {
  telemetry_msg_head_t head;
  uint8_t mode;
  uint16_t vbus;
  uint16_t temp;
  uint16_t current;
  uint16_t speed;
} PACKED telemetry_msg_status_t;

typedef struct telemetry_msg_set_speed {
  telemetry_msg_head_t head;
  uint16_t speed;
  uint8_t rampup;
} PACKED telemetry_msg_set_speed_t;

typedef struct telemetry_msg_stop {
  telemetry_msg_head_t head;
} PACKED telemetry_msg_stop_t;

typedef struct telemetry_msg_start {
  telemetry_msg_head_t head;
} PACKED telemetry_msg_start_t;

void telemetry_init(void);
void telemetry_reset(void);
void telemetry_process_byte(uint8_t ch);
void telemetry_send_byte(uint8_t ch);
void telemetry_send_bytes(uint8_t *bytes, size_t len);
void telemetry_msg_set_speed_callback(telemetry_msg_set_speed_t *set_speed);
void telemetry_msg_stop_callback(telemetry_msg_stop_t *stop);
void telemetry_msg_start_callback(telemetry_msg_start_t *start);
void telemetry_msg_status_send(uint8_t mode, uint16_t vbus, uint16_t temp, uint16_t current, uint16_t speed);

#endif /* INC_TELEMETRY_H_ */
