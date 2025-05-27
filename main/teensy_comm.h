/**
 * @file teensy_comm.h
 *
 * @brief message structures for the teensy command and status messages.
 *
 * @author Riley Mark
 * @author February 27, 2025
 *
 ******************************************************************************/

#ifndef TEENSY_COMM_H
#define TEENSY_COMM_H

#include <stdint.h>
#include <cstddef>

#define NUM_JOINTS 8
#define NUM_EJCT_JOINTS 2
#define RAD_PER_ENC_STEP 0.0314159265359 // TODO: Rotation measurement accuracy test
#define ENC_STEP_PER_RAD 31.8309886184 // TODO: Rotation measurement accuracy test
#define CRC_POLY 1021
#define CRC_INIT 0xFFFF 


/**
 * This is packed so that there is no padding around the byte boundaries. This
 * ensures reliable parsing on both sides of the serial communication pipe.
 */  
#pragma pack(1)

typedef struct _teensy_header_t
{
    uint16_t header;
    uint16_t seq;
    uint16_t len;
    uint16_t type;
} teensy_header_t;

/**
 * Teensy command to set arm positions.
 * 
 * type = 0x01
 */
typedef struct _teensy_command_t
{
    teensy_header_t hdr;

    int16_t setpoint_position[NUM_JOINTS];
    uint16_t led_state;

    uint16_t crc;
} teensy_command_t;

/**
 * Teensy command to run the "zeroing" sequence
 * 
 * type = 0x02
 */
typedef struct _teensy_zero_command_t
{
    teensy_header_t hdr;

    uint16_t crc;
} teensy_zero_t;

/**
 * Teensy system status message  
 */
typedef struct _teensy_status_t
{
    teensy_header_t hdr;

    int16_t encoder[NUM_JOINTS];
    uint16_t debug_feild_0;
    uint16_t debug_feild_1;

    uint16_t crc;
} teensy_status_t;

#pragma pack()

/**
 * Converts encoder steps to radians
 * 
 * @param enc_steps - raw encoder values
 * @param gear_ratio - the gear ratio of the joint
 * 
 * @return - angle in radians
 */
double enc_steps_to_rad (uint32_t enc_steps, uint16_t gear_ratio);

/**
 * Converts encoder steps to radians
 * 
 * @param rad - angle in radians
 * @param gear_ratio - the gear ratio of the joint
 * 
 * @return - angle converted to encoder steps
 */
uint32_t rad_to_enc_steps (double rad, uint16_t gear_ratio);

/**
 * CRC16 -- Google this. It's one mechanism that allows data to be transferred
 * between devices and across networks without loosing integrity.
 * 
 * @param data - character buffer of message being sent
 * @param length - length of the data buffer
 * 
 * @return - unsigned 16 bit crc calculated on the entire payload (minus crc obv)
 */
uint16_t crc16_ccitt (const uint8_t *data, size_t length);

#endif