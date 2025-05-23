/**
 * @file teensy_comm.h
 *
 * @brief message structures for the teensy command and status messages.
 *
 * @author Riley Mark
 * @author Febuary 27, 2025
 *
 ******************************************************************************/

#include "teensy_comm.h"

/**
 * Converts encoder steps to radians
 * 
 * @param enc_steps - raw encoder values
 * @param gear_ratio - the gear ratio of the joint
 * 
 * @return - angle in radians
 */
double enc_steps_to_rad (uint32_t enc_steps, uint16_t gear_ratio)
{
    return enc_steps * gear_ratio * RAD_PER_ENC_STEP;
}

/**
 * Converts encoder steps to radians
 * 
 * @param rad - angle in radians
 * @param gear_ratio - the gear ratio of the joint
 * 
 * @return - angle converted to encoder steps
 */
uint32_t rad_to_enc_steps (double rad, uint16_t gear_ratio)
{
    return rad * gear_ratio * ENC_STEP_PER_RAD;
}

/**
 * CRC16 -- Google this. It's one mechanism that allows data to be transferred
 * between devices and across networks without loosing integrity.
 * 
 * @param data - character buffer of message being sent
 * @param length - length of the data buffer
 * 
 * @return - unsigned 16 bit crc calculated on the entire payload (minus crc obv)
 */
uint16_t crc16_ccitt (const uint8_t *data, size_t length)
{
    uint16_t crc = CRC_INIT;

    for (size_t ii = 0; ii < length; ii++) {
        crc ^= (data[ii] << 8); 

        for (int j = 0; j < 8; j++) 
        {
            if (crc & 0x8000) 
            {
                crc = (crc << 1) ^ CRC_POLY;
            } 
            else 
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}