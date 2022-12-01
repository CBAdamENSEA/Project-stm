/*
 * XL_320.h
 *
 *  Created on: Oct 22, 2022
 *      Author: cheik
 */

#ifndef INC_XL_320_H_
#define INC_XL_320_H_


#include <stdio.h>
#include <stdint.h>
#include "cmsis_os.h"


//// EEPROM
#define XL320_MODEL_NUMBER 0
#define XL320_FIRMWARE_VERSION 2
#define XL320_ID 3
#define XL320_BAUD_RATE 4
#define XL320_RETURN_DELAY_TIME 5
#define XL320_CW_ANGLE_LIMIT 6   // min angle, default 0
#define XL320_CCW_ANGLE_LIMIT 8   // max angle, default 300
#define XL320_CONTROL_MODE 11  // joint or wheel mode, default joint (servo)
#define XL320_TEMPERATURE_LIMIT 12
#define XL320_MIN_VOLTAGE_LIMIT 13
#define XL320_MAX_VOLTAGE_LIMIT 14
#define XL320_MAX_TORQUE 15
#define XL320_STATUS_RETURN_LEVEL 17
#define XL320_SHUTDOWN 18

//// RAM
#define XL320_TORQUE_ENABLE 24  // servo mode on/off - turn into wheel
#define XL320_LED 25
#define XL320_D_GAIN 27
#define XL320_I_GAIN 28
#define XL320_P_GAIN 29
#define XL320_GOAL_POSITION 30
#define XL320_MOVING_SPEED 32
#define XL320_TORQUE_LIMIT 35
#define XL320_PRESENT_POSITION 37  // current servo angle
#define XL320_PRESENT_SPEED 39  // current speed
#define XL320_PESENT_LOAD 41  // current load
#define XL320_PESENT_VOLTAGE 45  // current voltage
#define XL320_PESENT_TEMPERATURE 46  // current temperature
#define XL320_REGISTERED_INSTRUCTION 47
#define XL320_MOVING 49
#define XL320_HARDWARE_ERROR_STATUS 50
#define XL320_PUNCH 51

//// OTHER
#define XL320_RESET_ALL 0xFF
#define XL320_RESET_ALL_BUT_ID 0x01
#define XL320_RESET_ALL_BUT_ID_BAUD_RATE 0x02
#define XL320_LED_WHITE 7
#define XL320_LED_BLUE_GREEN 6
#define XL320_LED_PINK 5
#define XL320_LED_BLUE 4
#define XL320_LED_YELLOW 3
#define XL320_LED_GREEN 2
#define XL320_LED_RED 1
#define XL320_LED_OFF 0
#define XL320_BROADCAST_ADDR 0xfe  // a packet with this ID will go to all servos
#define XL320_WHEEL_MODE 1
#define XL320_JOINT_MODE 2  // normal servo
#define XL320_9600 0  // 0: 9600, 1:57600, 2:115200, 3:1Mbps
#define XL320_57600 1
#define XL320_115200 2
#define XL320_1000000 3

typedef struct{
	uint8_t id;
	uint8_t torque;
	uint16_t position;
	uint16_t speed;
	SemaphoreHandle_t sem_packet_read; //in HAL_UART_RxCpltCallback
} servo_t;

#define BUFFER (buffer)
#define BUFFER_INDEX (buffer_index)
#define BUFFER_LENGTH (128)

#define PARAMS_LENGTH 10

typedef enum
{
	Br_9600 = 0,
	Br_57600 = 1,
	Br_115200=2,
	Br_1000000=3
} XL320_baudrates;

//// Instructions
typedef enum
{
  ping = 0x01,
  read = 0x02,
  write = 0x03,

  reg_write = 0x04,
  action = 0x05,

  factory_reset = 0x06,
  reboot = 0x08,
  clear = 0x10,
  control_table_backup = 0x20,

  sync_read = 0x82,
  sync_write = 0x83,
  fast_sync_read = 0x8A,

  bulk_read = 0x92,
  bulk_write = 0x93,
  fast_bulk_read = 0x9A
} XL320_instructions;


//// Functions

uint16_t XL_320_CRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
void XL_320_Display_Packet(uint8_t * packet,uint8_t size);

void XL_320_send_packet(uint8_t id, XL320_instructions inst, uint8_t *params, uint16_t params_length, uint8_t Rx_packet_length);

uint8_t XL_320_ping(uint8_t id,uint16_t *model_number,uint8_t *firmware_version);
void XL_320_write(uint8_t id, uint16_t address, uint8_t *data, uint16_t data_length);
uint8_t XL_320_read(uint8_t id, uint16_t address, uint16_t data_length, uint8_t *return_data, uint16_t *return_data_length,uint8_t Rx_packet_length);

uint16_t XL_320_read_present_position(uint8_t id);
void XL_320_set_goal_position(uint8_t id, uint16_t position);
void XL_320_set_speed_position(uint8_t id, uint16_t speed);
void XL_320_set_torque_enable(uint8_t id, uint8_t enable);
void XL_320_set_baudrate(uint8_t id, uint8_t br);

void XL_320_clear_receive_buffer(void);
uint8_t XL_320_get_status_packet(uint8_t *packet, uint16_t *packet_length);
uint8_t XL_320_parse_status_packet(uint8_t *packet, uint32_t packet_length, uint8_t *id, uint8_t *params, uint16_t *params_length, uint8_t *error, uint8_t *crc_check);




#endif /* INC_XL_320_H_ */
