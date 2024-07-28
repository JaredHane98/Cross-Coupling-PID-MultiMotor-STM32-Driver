#include "I2C_Controller.h"
#include <string.h>
#include <stdint.h>





void Reset_I2C_Read_Message(I2C_Read_Message* message)
{
#if DEBUG_MODE
    if(message == NULL)
        Error_Handler();
#endif
    memset(message, 0, sizeof(I2C_Read_Message));
}

void Reset_I2C_Write_Message(I2C_Write_Message* message)
{
#if DEBUG_MODE
    if(message == NULL)
        Error_Handler();
#endif
    memset(message, 0, sizeof(I2C_Write_Message));
}

void Reset_I2C_CRC_Message(I2C_CRC_Message* message)
{
#if DEBUG_MODE
    if(message == NULL)
        Error_Handler();
#endif
    memset(message, 0, sizeof(I2C_CRC_Message));
}




void Reinit_I2C(I2C_Handle* handle, I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
    //Reset_I2C_Read_Message(&handle->current_read_message);
    Reset_I2C_Write_Message(&handle->write_message); 
    Reset_I2C_CRC_Message(&handle->crc_message);
    handle->message_start_time = 0;
    handle->message_type = NO_MESSAGE;
}

void Process_Ack_I2C(I2C_Handle* handle)
{
    if(handle->transfer_direction == I2C_DIRECTION_TRANSMIT)
    {
        if(handle->message_type == READ_MESSAGE)
        {
            if(handle->current_read_message.index == I2C_MESSAGE_SIZE)
            {
                handle->message_type = NO_MESSAGE;
                //Reset_I2C_Read_Message(&handle->current_read_message);
            }
        }
    }
}

void I2C_ErrorCallback(I2C_Handle* handle, I2C_HandleTypeDef *hi2c)
{
#if DEBUG_MODE
    if(handle == NULL || hi2c == NULL)
        Error_Handler();
#endif
    switch(HAL_I2C_GetError(hi2c))
    {
        case HAL_I2C_ERROR_AF:
            Process_Ack_I2C(handle);
            break;
        case HAL_I2C_ERROR_BERR:
            Reinit_I2C(handle, hi2c);
            break;
        default:
            Error_Handler();
    }

    if(HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) // State must be ready for HAL_I2C_EnableListen_IT() to work. Resetting the controller sets it.
        Reinit_I2C(handle, hi2c);
    
    if(HAL_I2C_EnableListen_IT(hi2c) != HAL_OK)
        Error_Handler();
}







uint32_t addr_callback_index = 0;
uint32_t rx_callback_index = 0;

void I2C_AddrCallback(I2C_Handle* handle, I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
#if DEBUG_MODE
    if(handle == NULL || hi2c == NULL)
        Error_Handler(); 
#endif    
    if(AddrMatchCode == I2C_ADDR)
    {
        handle->transfer_direction = TransferDirection;
        handle->message_start_time = HAL_GetTick();

        addr_callback_index++;

        if(handle->transfer_direction == I2C_DIRECTION_TRANSMIT)
        {
        	handle->current_message = 0;
            handle->message_type = NO_MESSAGE;

            if(HAL_I2C_Slave_Seq_Receive_IT(hi2c, &handle->current_message, 1, I2C_FIRST_FRAME) != HAL_OK)
                Error_Handler();
        }
        else
        {
            switch(handle->message_type)
            {
                case READ_MESSAGE:
                    if(HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &handle->current_read_message.message[0], I2C_MESSAGE_SIZE, I2C_LAST_FRAME) != HAL_OK)
                        Error_Handler();
                    handle->current_read_message.index = 1;
                    break;
                case CRC_MESSAGE:
                    if(HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &handle->crc_message.message[0], I2C_MESSAGE_SIZE, I2C_LAST_FRAME) != HAL_OK)
                        Error_Handler();
                    handle->crc_message.index = 1;
                    break;
                default:
                    Error_Handler();
            }
        }
    }
}


void Set_Next_Motor_Command(const uint8_t command, const uint8_t *data, I2C_Motor_Command* command_handle)
{
    command_handle->next_command = command;
    memcpy(command_handle->data, data, sizeof(uint8_t) * 4);
}


void Set_Motor_Value(Motor_Handle* motor_handle, const uint8_t value_name, const uint8_t *data)
{
    const float value = *(const float*)(data);
    switch(value_name)
    {
        // case PID_KP1:
        //     motor_handle->front_left_motor.pid_position.kp = value;
        //     break;
        // case PID_KI1:
        //     motor_handle->front_left_motor.pid_position.ki = value;
        //     break;
        // case PID_KD1:
        //     motor_handle->front_left_motor.pid_position.kd = value;
        //     break;
        // case PID_MIN1:
        //     motor_handle->front_left_motor.pid_position.min = value;
        //     break;
        // case PID_MAX1:
        //     motor_handle->front_left_motor.pid_position.max = value;
        //     break;

        // case PID_KP2:
        //     motor_handle->front_right_motor.pid_position.kp = value;
        //     break;
        // case PID_KI2:
        //     motor_handle->front_right_motor.pid_position.ki = value;
        //     break;
        // case PID_KD2:
        //     motor_handle->front_right_motor.pid_position.kd = value;
        //     break;
        // case PID_MIN2:
        //     motor_handle->front_right_motor.pid_position.min = value;
        //     break;
        // case PID_MAX2:
        //     motor_handle->front_right_motor.pid_position.max = value;
        //     break;

        // case PID_KP3:
        //     motor_handle->back_left_motor.pid_position.kp = value;
        //     break;
        // case PID_KI3:
        //     motor_handle->back_left_motor.pid_position.ki = value;
        //     break;
        // case PID_KD3:
        //     motor_handle->back_left_motor.pid_position.kd = value;
        //     break;
        // case PID_MIN3:
        //     motor_handle->back_left_motor.pid_position.min = value;
        //     break;
        // case PID_MAX3:
        //     motor_handle->back_left_motor.pid_position.max = value;
        //     break;

        // case PID_KP4:
        //     motor_handle->back_right_motor.pid_position.kp = value;
        //     break;
        // case PID_KI4:
        //     motor_handle->back_right_motor.pid_position.ki = value;
        //     break;
        // case PID_KD4:
        //     motor_handle->back_right_motor.pid_position.kd = value;
        //     break;
        // case PID_MIN4:
        //     motor_handle->back_right_motor.pid_position.min = value;
        //     break;
        // case PID_MAX4:
        //     motor_handle->back_right_motor.pid_position.max = value;
        //     break;
        case TARGET_POSITON: 
            motor_handle->front_left_motor.state.steps = 0; // Assuming the priority and sub-prority are correct this should be safe.
            motor_handle->front_right_motor.state.steps = 0;
            motor_handle->back_left_motor.state.steps = 0;
            motor_handle->back_right_motor.state.steps = 0;

            
            motor_handle->front_left_motor.data.updated_position = value;  // This part is fine.
            motor_handle->front_right_motor.data.updated_position = value;
            motor_handle->back_left_motor.data.updated_position = value;
            motor_handle->back_right_motor.data.updated_position = value;
            break;
        case TARGET_VELOCITY:
            break; // todo
        default:
            Error_Handler(); 
            break;
    }
}



uint32_t I2C_Compute_CRC(uint8_t* data, uint8_t length)
{
    const uint32_t polynomial = 0xEDB88320;
    uint8_t* current  = data;
    uint32_t current_crc = 0;

    while(length--)
    {
        current_crc ^= *current++;
        for(uint8_t j = 0; j < 8; j++)
            current_crc = (current_crc >> 1) ^ (-(int32_t)(current_crc & 1) & polynomial);
    }
    current_crc = ~current_crc;
    return current_crc;
}




void Process_I2C_Write_Message(Motor_Handle* motor_handle, I2C_Handle* i2c_handle, I2C_Motor_Command* command_handle)
{
    const uint32_t computed_crc = I2C_Compute_CRC(i2c_handle->write_message.message, I2C_WRITE_DATA_SIZE);
    const uint8_t message = i2c_handle->write_message.addr;
    const uint8_t* data = i2c_handle->write_message.data;

    if(computed_crc == i2c_handle->write_message.crc)
    {
        if(Is_Valid_Motor_Command(message))
            Set_Next_Motor_Command(message, data, command_handle);
        else if(Is_Valid_Value(message))
            Set_Motor_Value(motor_handle, message, data);
        else
            Error_Handler();
    }

    i2c_handle->crc_message.index = 0;
    i2c_handle->crc_message.crc = computed_crc;
    i2c_handle->crc_message.addr = message;
    i2c_handle->write_message.index = 0;
}

uint32_t in_use_index = 0;
uint32_t updated_index = 0;
void I2C_SlaveRxCpltCallback(Motor_Handle* motor_handle, I2C_Handle* handle, I2C_Motor_Command* command_handle, I2C_HandleTypeDef* hi2c)
{
#if DEBUG_MODE
    if(handle == NULL || hi2c == NULL)
        Error_Handler(); 
#endif 
    if(handle->transfer_direction != I2C_DIRECTION_TRANSMIT)
        Error_Handler();


    switch(handle->current_message)
    {
        case READ_COMMAND:
            if(!handle->next_read_message.in_use)
            {
                handle->next_read_message.in_use = true;
                if(handle->next_read_message.updated)
                {
                    memcpy(handle->current_read_message.message, handle->next_read_message.message, I2C_MESSAGE_SIZE);
                    handle->next_read_message.updated = false;
                }
                handle->next_read_message.in_use = false;
            }
            handle->current_read_message.index = 0;
            handle->message_type = READ_MESSAGE;
            break;
        case CRC_COMMAND:
            handle->message_type = CRC_MESSAGE;
            break;
        default:
            if(handle->message_type == NO_MESSAGE)
            {
                handle->message_type = WRITE_MESSAGE;
                handle->write_message.index = 0;
                handle->write_message.message[handle->write_message.index] = handle->current_message;
                handle->write_message.index++;
            }
            if(handle->write_message.index == I2C_WRITE_SIZE-1)
            {
                if(HAL_I2C_Slave_Seq_Receive_IT(hi2c, &handle->write_message.message[handle->write_message.index], 1, I2C_LAST_FRAME) != HAL_OK)
                    Error_Handler();
            }
            else
            {
                if(HAL_I2C_Slave_Seq_Receive_IT(hi2c, &handle->write_message.message[handle->write_message.index], 1, I2C_NEXT_FRAME) != HAL_OK)
                    Error_Handler();
            }
            handle->write_message.index++;

            if(handle->write_message.index == I2C_WRITE_SIZE)
                Process_I2C_Write_Message(motor_handle, handle, command_handle);
            break;
    }




    // if(HAL_I2C_Slave_Seq_Receive_IT(hi2c, &handle->write_message.message[handle->write_message.index], 1, I2C_NEXT_FRAME) != HAL_OK)
        //Error_Handler();
    

    if(handle->write_message.index == I2C_WRITE_SIZE)
        Process_I2C_Write_Message(motor_handle, handle, command_handle);
}


void I2C_SlaveTxCpltCallback(I2C_Handle* handle, I2C_HandleTypeDef* hi2c)
{
#if DEBUG_MODE
    if(handle == NULL || hi2c == NULL)
        Error_Handler();
#endif
    if(handle->transfer_direction != I2C_DIRECTION_RECEIVE)
        Error_Handler();
    
    switch(handle->message_type)
    {
        case READ_MESSAGE:
            handle->current_read_message.index++;
            break;
        case CRC_MESSAGE:
            handle->crc_message.index++;
            break;
        default:
            Error_Handler();
            break;
    }
}


void I2C_ListenCpltCallback(I2C_Handle* handle, I2C_HandleTypeDef* hi2c)
{
#if DEBUG_MODE
    if(hi2c == NULL || hi2c == NULL)
        Error_Handler();
#endif
    if(HAL_I2C_EnableListen_IT(hi2c) != HAL_OK)
        Error_Handler();
}

void Init_I2C_Handle(I2C_Handle* handle, I2C_HandleTypeDef* i2c_handle)
{
#if DEBUG_MODE
    if(handle == NULL || i2c_handle == NULL)
        Error_Handler();
#endif 
    memset(handle, 0, sizeof(I2C_Handle));

    if(HAL_I2C_EnableListen_IT(i2c_handle) != HAL_OK)
        Error_Handler();
}   


