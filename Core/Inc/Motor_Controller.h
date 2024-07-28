#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H
#include "main.h"
#include "Motor.h"


#define MCP4728_CMD_FASTWRITE		0x00
#define MCP4728_CMD_DACWRITE_MULTI	0x40
#define MCP4728_CMD_DACWRITE_SEQ	0x50
#define MCP4728_CMD_DACWRITE_SINGLE	0x58
#define MCP4728_CMD_ADDRWRITE		0x60
#define	MCP4728_CMD_VREFWRITE		0x80
#define MCP4728_CMD_GAINWRITE		0xC0
#define MCP4728_CMD_PWRDWNWRITE		0xA0

#define MCP4728_BASEADDR			0x60<<1

#define MCP4728_VREF_VDD			0x0
#define MCP4728_VREF_INTERNAL		0x1

#define MCP4728_GAIN_1				0x0
#define MCP4728_GAIN_2				0x1

#define MCP4728_CHANNEL_A			0x0
#define MCP4728_CHANNEL_B			0x1
#define MCP4728_CHANNEL_C			0x2
#define MCP4728_CHANNEL_D			0x3

#define MCP4728_PWRDWN_NORMAL		0x0
#define MCP4728_PWRDWN_1			0x1
#define MCP4728_PWRDWN_2			0x2
#define MCP4728_PWRDWN_3			0x3

#define MCP4728_UDAC_UPLOAD			0x1
#define MCP4728_UDAC_NOLOAD			0x0

#define MCP4728_GENERAL_RESET		0x06
#define MCP4728_GENERAL_WAKEUP		0x09
#define MCP4728_GENERAL_SWUPDATE	0x08
#define MCP4728_GENERAL_READADDR	0x0C

#define NUM_MOTORS (uint8_t)4

typedef struct
{
    union
    {
        struct
        {
            Motor motors[4];
        };
        struct
        {
            Motor front_left_motor;
            Motor front_right_motor;
            Motor back_left_motor;
            Motor back_right_motor;
        };
    };
    I2C_HandleTypeDef* i2c_handle;
}Motor_Handle;



typedef struct
{
    union
    {
        struct
        {
            uint8_t data[sizeof(float)];
        };
        float f_data;
    };
    motor_command_t current_command;
    motor_command_t next_command;
}I2C_Motor_Command; 



void Set_I2C_Handle(Motor_Handle* handle, I2C_HandleTypeDef* i2c_handle);

void Run_Stop_Command(Motor_Handle* handle, I2C_Motor_Command* motor_command);

bool Is_Position_Command_Done(Motor_Handle* handle);

bool Is_Velocity_Command_Done(Motor_Handle* handle, const float target_dist);

void Run_Forward_Position_Command(Motor_Handle* handle, I2C_Motor_Command* motor_command);

void Run_Backward_Position_Command(Motor_Handle* handle, I2C_Motor_Command* motor_command);


//// SYNCHRONIZING POSITION FUNCTIONS

void Run_Front_Position_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command);

void Run_Back_Position_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command);

void Run_Left_Position_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command);

void Run_Right_Position_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command);

void Run_Sync_All_Position(Motor_Handle* handle, I2C_Motor_Command* motor_command);



//// SYNCHRONIZING VELOCITY FUNCTIONS

void Run_Front_Velocity_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command);

void Run_Back_Velocity_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command);

void Run_Left_Velocity_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command);

void Run_Right_Velocity_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command);

void Run_Sync_All_Velocity(Motor_Handle* handle, I2C_Motor_Command* motor_command);

#endif // MOTOR_CONTROLLER
