#ifndef I2C_CONTROLLER_H
#define I2C_CONTROLLER_H
#include "main.h"
#include "Motor_Controller.h"
#include <stdint.h>
#include <stdbool.h>


#define I2C_ADDR (uint8_t)36
#define I2C_MESSAGE_SIZE (uint8_t)100
#define I2C_CRC_SIZE (uint8_t)5
#define I2C_WRITE_SIZE (uint8_t)9
#define I2C_WRITE_DATA_SIZE (uint8_t)5
#define I2C_MESSAGE_DATA_SIZE (uint8_t)96





typedef enum 
{
    NO_MESSAGE = 0,
    WRITE_MESSAGE,
    READ_MESSAGE,
    CRC_MESSAGE,
}i2c_message_t;


typedef struct 
{
    union
    {
        struct
        {
            uint8_t message[I2C_WRITE_SIZE]; 
        };
        struct
        {
            uint8_t addr;
            uint8_t data[4];
            uint32_t crc;
        };
    };
    uint8_t index;
}I2C_Write_Message;



typedef struct
{
    union
    {
        struct
        {
            uint8_t message[I2C_MESSAGE_SIZE];
        };
        struct
        {
            uint8_t padding[I2C_MESSAGE_DATA_SIZE];
            uint32_t crc;
        }; 
    }; 
    uint8_t index;
    volatile bool in_use;
    volatile bool updated;
}I2C_Read_Message; 

typedef struct
{
    union
    {
        struct
        {
            uint8_t message[I2C_CRC_SIZE];
        };
        struct
        {
            uint8_t addr;
            uint32_t crc;
        };
    };
    uint8_t index;
}I2C_CRC_Message;







typedef struct 
{
    I2C_Write_Message write_message;
    I2C_Read_Message current_read_message;
    I2C_Read_Message next_read_message;
    I2C_CRC_Message crc_message;
    uint32_t message_start_time;
    uint8_t transfer_direction; 
    uint8_t current_message;
    i2c_message_t message_type;
}I2C_Handle;


// typedef struct 
// {
//     I2C_Message transmit_message;
//     I2C_Message recieve_message;
//     I2C_Ready_Message ready_message;
//     I2C_Motor_Command* motor_command;
//     uint32_t msg_start_time;
//     uint8_t transfer_direction;
//     i2c_message_t message_type;
// }I2C_Handle;


/// @brief Resets I2C message
/// @param message To reset
void Reset_I2C_Read_Message(I2C_Read_Message* message);


/// @brief Resets I2C message
/// @param message To reset
void Reset_I2C_Write_Message(I2C_Write_Message* message);

/**
  * @brief  I2C user implemented error callback.
  * @param  handle Pointer to I2C_Handle 
  * @param  hi2c Pointer to a I2C_HandleTypeDef 
  */
void I2C_ErrorCallback(I2C_Handle* handle, I2C_HandleTypeDef *hi2c);


/**
  * @brief  I2C user implemented addr callback. Used to intiate a transfer
  * @param  handle Pointer to I2C_Handle 
  * @param  hi2c Pointer to a I2C_HandleTypeDef 
  * @param  transferDirection direction of the transfer
  * @param  AddrMatchCode Address match code
  */
void I2C_AddrCallback(I2C_Handle* handle, I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);


/**
  * @brief  I2C user implemented recieve callback. Used to determine recieve completeness
  * @param  handle Pointer to I2C_Handle 
  * @param  hi2c Pointer to a I2C_HandleTypeDef 
  */
void I2C_SlaveRxCpltCallback(Motor_Handle* motor_handle, I2C_Handle* handle, I2C_Motor_Command* command_handle, I2C_HandleTypeDef* hi2c);


/**
  * @brief  I2C user implemented recieve callback. Used to determine transmit completeness
  * @param  handle Pointer to I2C_Handle 
  * @param  hi2c Pointer to a I2C_HandleTypeDef 
  */
void I2C_SlaveTxCpltCallback(I2C_Handle* handle, I2C_HandleTypeDef* hi2c);


/**
  * @brief  Enables listening on the I2C device
  * @param  handle Pointer to I2C_Handle 
  * @param  hi2c Pointer to a I2C_HandleTypeDef 
  */
void I2C_ListenCpltCallback(I2C_Handle* handle, I2C_HandleTypeDef* hi2c);


/**
 *  @brief  Initializes I2C_Handle
 *  @param  handle Pointer to I2C_Handle
*/
void Init_I2C_Handle(I2C_Handle* handle, I2C_HandleTypeDef* i2c_handle);


/**
  * @brief  Initializes I2C_Handle
  * @param  data Pointer to Data
  * @param  length Length of data
  * @return  computed CRC
*/
uint32_t I2C_Compute_CRC(uint8_t* data, uint8_t length);


#endif // I2C_CONTROLLER_H
