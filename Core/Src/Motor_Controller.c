#include "Motor_Controller.h"


#define ABS(x) ((x) > 0 ? (x) : -(x))
#define MINIMUM_DIST 0.01 




HAL_StatusTypeDef MCP4728_Write_GeneralCall(I2C_HandleTypeDef *I2CHandler, uint8_t command)
{
	return HAL_I2C_Master_Transmit(I2CHandler, 0x00, &command, 1, HAL_MAX_DELAY);
}


void MCP4728_Init(I2C_HandleTypeDef *i2c_handle)
{
	if(MCP4728_Write_GeneralCall(i2c_handle, MCP4728_GENERAL_RESET) != HAL_OK)
		Error_Handler();

	if(MCP4728_Write_GeneralCall(i2c_handle, MCP4728_GENERAL_WAKEUP) != HAL_OK)
		Error_Handler();

	uint8_t buf[9];
	buf[0] = MCP4728_CMD_DACWRITE_SEQ;
	for(uint8_t i = 1; i <= 4; i++){
		buf[(i*2)+1] = 0x00;
		buf[(i*2)] = (0 << 7) | ((i-1)<<4) | 0x0;
	}
	if(HAL_I2C_Master_Transmit(i2c_handle, MCP4728_BASEADDR, buf, sizeof(buf), HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();

	if(MCP4728_Write_GeneralCall(i2c_handle,MCP4728_GENERAL_SWUPDATE) != HAL_OK)
		Error_Handler();
}


void Set_I2C_Handle(Motor_Handle* handle, I2C_HandleTypeDef* i2c_handle)
{
	handle->i2c_handle = i2c_handle;
	//MCP4728_Init(i2c_handle);
}




/*
 * If the code fails and we cannot communicate with the device we CANNOT stop the motors
 * We must have some way to implement a way to shutdown the motors without using I2C
 */
void Set_All_DAC(Motor_Handle *handle, uint16_t output)
{
	uint8_t buf[3];
	for(uint8_t i = 0; i < NUM_MOTORS; i++)
	{
		handle->motors[i].pins.dac.dac_value = output;

        uint8_t sequential_write_cmd = MCP4728_CMD_DACWRITE_MULTI;
        sequential_write_cmd |= (handle->motors[i].pins.dac.channel << 1);

	    buf[0] = sequential_write_cmd;
	    buf[1] = output >> 8;
	    buf[2] = output & 0xFF;

	    if(HAL_I2C_Master_Transmit(handle->i2c_handle, MCP4728_BASEADDR, buf, sizeof(buf), HAL_MAX_DELAY) != HAL_OK)
	    	Error_Handler();
	}
}



void Update_Multi_DAC(Motor_Handle* handle, const uint8_t* indices, const uint8_t size)
{
    uint8_t buf[3];
    for(uint8_t i = 0; i < size; i++)
    {
        uint8_t sequential_write_cmd = MCP4728_CMD_DACWRITE_MULTI;
        sequential_write_cmd |= (handle->motors[indices[i]].pins.dac.channel << 1);

        buf[0] = sequential_write_cmd;
        buf[1] = handle->motors[indices[i]].pins.dac.dac_value >> 8;
        buf[2] = handle->motors[indices[i]].pins.dac.dac_value & 0xFF;

        if(HAL_I2C_Master_Transmit(handle->i2c_handle, MCP4728_BASEADDR, buf, sizeof(buf), HAL_MAX_DELAY) != HAL_OK)
            Error_Handler();
    }
}

void Update_All_DAC(Motor_Handle* handle)
{
	uint8_t buf[3];
	for(uint8_t i = 0; i < NUM_MOTORS; i++)
	{
        uint8_t sequential_write_cmd = MCP4728_CMD_DACWRITE_MULTI;
        sequential_write_cmd |= (handle->motors[i].pins.dac.channel << 1);

	    buf[0] = sequential_write_cmd;
	    buf[1] = handle->motors[i].pins.dac.dac_value >> 8;
	    buf[2] = handle->motors[i].pins.dac.dac_value & 0xFF;

	    if(HAL_I2C_Master_Transmit(handle->i2c_handle, MCP4728_BASEADDR, buf, sizeof(buf), HAL_MAX_DELAY) != HAL_OK)
	    	Error_Handler();
	}
}
void Set_DAC_Default(Motor_Handle* handle)
{
    uint8_t buf[3];
    for(uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        handle->motors[i].pins.dac.dac_value = handle->motors[i].pid_position.dac_default;
        uint8_t sequential_write_cmd = MCP4728_CMD_DACWRITE_MULTI;
        sequential_write_cmd |= (handle->motors[i].pins.dac.channel << 1);

        buf[0] = sequential_write_cmd;
	    buf[1] = handle->motors[i].pins.dac.dac_value >> 8;
	    buf[2] = handle->motors[i].pins.dac.dac_value & 0xFF;

        if(HAL_I2C_Master_Transmit(handle->i2c_handle, MCP4728_BASEADDR, buf, sizeof(buf), HAL_MAX_DELAY) != HAL_OK)
	    	Error_Handler();
    }
}



void Run_Stop_Command(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    Set_DAC_Default(handle);

    for(uint8_t i = 0; i < NUM_MOTORS; i++)
        Set_Motor_Brake(&handle->motors[0], GPIO_PIN_SET);

    motor_command->current_command = STOP_COMMAND;
}






// void Fault_Stop_Motors()
// {
// 	PWM_Set_Duty_Cycle(&front_left_motor.pins.pwm, 0);
// 	PWM_Set_Duty_Cycle(&front_right_motor.pins.pwm, 0);
// 	PWM_Set_Duty_Cycle(&back_left_motor.pins.pwm, 0);
// 	PWM_Set_Duty_Cycle(&back_right_motor.pins.pwm, 0);

// 	Set_Motor_Brake(&front_left_motor, GPIO_PIN_SET);
// 	Set_Motor_Brake(&front_right_motor, GPIO_PIN_SET);
// 	Set_Motor_Brake(&back_left_motor, GPIO_PIN_SET);
// 	Set_Motor_Brake(&back_right_motor, GPIO_PIN_SET);
// }

bool Is_Position_Command_Done(Motor_Handle* handle)
{
    for(uint8_t i = 0; i < NUM_MOTORS;i++)
        if(handle->motors[i].data.position >= handle->motors[i].data.target)
            return true;

    for(uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        const float dist = handle->motors[i].data.position - handle->motors[i].data.target;
        if(ABS(dist) < MINIMUM_DIST)
            return true;
    }

    return false;
}

bool Is_Velocity_Command_Done(Motor_Handle* handle, const float target_dist)
{
    for(uint8_t i = 0; i < NUM_MOTORS; i++)
        if(handle->motors[i].data.position >= target_dist)
            return true;

    for(uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        const float dist = handle->motors[i].data.position - target_dist;
        if(ABS(dist) < MINIMUM_DIST)
            return true;
    }

    return false;
}



void Run_Forward_Position_Command(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    for(uint8_t i = 0; i < NUM_MOTORS; i++)
        Set_Motor_Brake(&handle->motors[i], GPIO_PIN_RESET);

    Set_Motor_Direction(&handle->front_left_motor, GPIO_PIN_SET);
    Set_Motor_Direction(&handle->front_right_motor, GPIO_PIN_RESET);

    Set_Motor_Direction(&handle->back_left_motor, GPIO_PIN_SET);
    Set_Motor_Direction(&handle->back_right_motor, GPIO_PIN_RESET);

    for(uint8_t i = 0; i < NUM_MOTORS; i++)
        Calculate_Motor_Position(&handle->motors[i]);

    if(Is_Position_Command_Done(handle))
    {
        Run_Stop_Command(handle, motor_command);

        for(uint8_t i = 0; i < NUM_MOTORS; i++)
            Calculate_Motor_Alignment(&handle->motors[i]);
    }
    else
    {
        for(uint8_t i = 0; i < NUM_MOTORS; i++)
            Update_Motor_DAC_PID_Position(&handle->motors[i]);

        Update_All_DAC(handle);
    }
}   



void Run_Backward_Position_Command(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    for(uint8_t i = 0; i < NUM_MOTORS; i++)
        Set_Motor_Brake(&handle->motors[i], GPIO_PIN_RESET);

    Set_Motor_Direction(&handle->front_left_motor, GPIO_PIN_RESET);
    Set_Motor_Direction(&handle->front_right_motor, GPIO_PIN_SET);

    Set_Motor_Direction(&handle->back_left_motor, GPIO_PIN_RESET);
    Set_Motor_Direction(&handle->back_right_motor, GPIO_PIN_SET);

    for(uint8_t i = 0; i < NUM_MOTORS; i++)
        Calculate_Motor_Position(&handle->motors[i]);

    if(Is_Position_Command_Done(handle))
    {
        Run_Stop_Command(handle, motor_command);

        for(uint8_t i = 0; i < NUM_MOTORS; i++)
            Calculate_Motor_Alignment(&handle->motors[i]);
    }
    else
    {
        for(uint8_t i = 0; i < NUM_MOTORS; i++)
            Update_Motor_DAC_PID_Position(&handle->motors[i]);
        
        Update_All_DAC(handle);
    }
}


void Run_Front_Position_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    const float SYNC_DISTANCE = 5.0f;
    uint8_t update_indices[2]  = {0, 1};

    handle->front_right_motor.data.position = 0;
    handle->front_right_motor.data.target = 0xFF;

    handle->back_right_motor.data.position = 0; 
    handle->back_right_motor.data.target = 0xFF;

    Set_Motor_Command(&handle->front_left_motor, &handle->front_right_motor, false, SYNC_DISTANCE);
    Set_Motor_Command(&handle->front_right_motor, &handle->front_left_motor, true, SYNC_DISTANCE);

    Set_Motor_Brake(&handle->front_left_motor, GPIO_PIN_RESET);
    Set_Motor_Brake(&handle->front_right_motor, GPIO_PIN_RESET);

    Set_Motor_Direction(&handle->front_left_motor, GPIO_PIN_RESET);
    Set_Motor_Direction(&handle->front_right_motor, GPIO_PIN_SET);

    while(true)
    {
        Calculate_Motor_Position(&handle->front_left_motor);
        Calculate_Motor_Position(&handle->front_right_motor);

        if(Is_Position_Command_Done(handle))
        {
            Run_Stop_Command(handle, motor_command);

            Calculate_Motor_Alignment(&handle->front_left_motor);
            Calculate_Motor_Alignment(&handle->front_right_motor);

            break;
        }
        else
        {
            Update_Motor_DAC_PID_Position(&handle->front_left_motor);
            Update_Motor_DAC_PID_Position(&handle->front_right_motor);

            Update_Multi_DAC(handle, update_indices, 2);
        }
    }
}


void Run_Back_Position_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    const float SYNC_DISTANCE = 5.0f;
    uint8_t update_indices[2] = {2, 3};

    handle->front_left_motor.data.position = 0;
    handle->front_left_motor.data.target = 0xFF;

    handle->front_right_motor.data.position = 0;
    handle->front_right_motor.data.target = 0xFF;

    Set_Motor_Command(&handle->back_left_motor, &handle->back_right_motor, false, SYNC_DISTANCE);
    Set_Motor_Command(&handle->back_right_motor, &handle->back_left_motor, true, SYNC_DISTANCE);

    Set_Motor_Brake(&handle->back_left_motor, GPIO_PIN_RESET);
    Set_Motor_Brake(&handle->back_right_motor, GPIO_PIN_RESET);
   
    Set_Motor_Direction(&handle->back_left_motor, GPIO_PIN_RESET);
    Set_Motor_Direction(&handle->back_right_motor, GPIO_PIN_SET);

    while(true)
    {
        Calculate_Motor_Position(&handle->back_left_motor);
        Calculate_Motor_Position(&handle->back_right_motor);

        if(Is_Position_Command_Done(handle))
        {
            Run_Stop_Command(handle, motor_command);

            Calculate_Motor_Alignment(&handle->back_left_motor);
            Calculate_Motor_Alignment(&handle->back_right_motor);

            break;
        }
        else
        {
            Update_Motor_DAC_PID_Position(&handle->back_left_motor);
            Update_Motor_DAC_PID_Position(&handle->back_right_motor);

            Update_Multi_DAC(handle, update_indices, 2);
        }
    }
}

void Run_Left_Position_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    const float SYNC_DISTANCE = 5.0f;
    uint8_t update_indices[2] = {0, 2};

    handle->front_right_motor.data.position = 0;
    handle->front_right_motor.data.target = 0xFF;

    handle->back_right_motor.data.position = 0;
    handle->back_right_motor.data.target = 0xFF;

    Set_Motor_Command(&handle->front_left_motor, &handle->back_left_motor, false, SYNC_DISTANCE);
    Set_Motor_Command(&handle->back_left_motor, &handle->front_left_motor, true, SYNC_DISTANCE);

    Set_Motor_Brake(&handle->front_left_motor, GPIO_PIN_RESET);
    Set_Motor_Brake(&handle->back_left_motor, GPIO_PIN_RESET);

    Set_Motor_Direction(&handle->front_left_motor, GPIO_PIN_RESET);
    Set_Motor_Direction(&handle->back_left_motor, GPIO_PIN_RESET);

    while(true)
    {
        Calculate_Motor_Position(&handle->front_left_motor);
        Calculate_Motor_Position(&handle->back_left_motor);

        if(Is_Position_Command_Done(handle))
        {
            Run_Stop_Command(handle, motor_command);

            Calculate_Motor_Alignment(&handle->front_left_motor);
            Calculate_Motor_Alignment(&handle->back_left_motor);

            break;
        }
        else
        {
            Update_Motor_DAC_PID_Position(&handle->front_left_motor);
            Update_Motor_DAC_PID_Position(&handle->back_left_motor);

            Update_Multi_DAC(handle, update_indices, 2);
        }
    }
}


void Run_Right_Position_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    const float SYNC_DISTANCE = 5.0f;
    uint8_t update_indices[2] = {1, 3};

    handle->front_left_motor.data.position = 0;
    handle->front_left_motor.data.target = 0xFF;

    handle->back_left_motor.data.position = 0;
    handle->back_left_motor.data.target = 0xFF;

    Set_Motor_Command(&handle->front_right_motor, &handle->back_right_motor, false, SYNC_DISTANCE);
    Set_Motor_Command(&handle->back_right_motor, &handle->front_right_motor, true, SYNC_DISTANCE);

    Set_Motor_Brake(&handle->front_right_motor, GPIO_PIN_RESET);
    Set_Motor_Brake(&handle->back_right_motor, GPIO_PIN_RESET);

    Set_Motor_Direction(&handle->front_right_motor, GPIO_PIN_RESET);
    Set_Motor_Direction(&handle->back_right_motor, GPIO_PIN_RESET);

    while(true)
    {
        Calculate_Motor_Position(&handle->front_right_motor);
        Calculate_Motor_Position(&handle->back_right_motor);

        if(Is_Position_Command_Done(handle))
        {
            Run_Stop_Command(handle, motor_command);

            Calculate_Motor_Alignment(&handle->front_right_motor);
            Calculate_Motor_Alignment(&handle->back_right_motor);

            break;
        }
        else
        {
            Update_Motor_DAC_PID_Position(&handle->front_right_motor);
            Update_Motor_DAC_PID_Position(&handle->back_right_motor);

            Update_Multi_DAC(handle, update_indices, 2);
        }
    }
}


void Run_Sync_All_Position(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    const float SYNC_DISTANCE = 5.0f;

    Set_Motor_Command(&handle->front_left_motor, &handle->front_right_motor, false, SYNC_DISTANCE);
    Set_Motor_Command(&handle->front_right_motor, &handle->front_left_motor, true, SYNC_DISTANCE);
    Set_Motor_Command(&handle->back_left_motor, &handle->back_right_motor, false, SYNC_DISTANCE);
    Set_Motor_Command(&handle->back_right_motor, &handle->back_left_motor, true, SYNC_DISTANCE);

    for(uint8_t i = 0; i < NUM_MOTORS; i++)
        Set_Motor_Brake(&handle->motors[i], GPIO_PIN_RESET); 

    Set_Motor_Direction(&handle->front_left_motor, GPIO_PIN_SET);
    Set_Motor_Direction(&handle->front_right_motor, GPIO_PIN_RESET);

    Set_Motor_Direction(&handle->back_left_motor, GPIO_PIN_SET);
    Set_Motor_Direction(&handle->back_right_motor, GPIO_PIN_RESET);

    while(true)
    {

        for(uint8_t i = 0; i < NUM_MOTORS; i++)
            Calculate_Motor_Position(&handle->motors[i]);

        if(Is_Position_Command_Done(handle))
        {
            Run_Stop_Command(handle, motor_command);

            for(uint8_t i = 0; i < NUM_MOTORS; i++) 
                Calculate_Motor_Alignment(&handle->motors[i]);     

            break;
        }
        else
        {
            for(uint8_t i = 0; i < NUM_MOTORS; i++)
                Update_Motor_DAC_PID_Position(&handle->motors[i]);
        
            Update_All_DAC(handle);
        }
    }
}

void Run_Front_Velocity_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    const float SYNC_VELOCITY_RPM = 300;
    const float VELOCITY_POSITION_DISTANCE = 10.0f;
    uint8_t update_indices[2] = {0, 1};

    handle->back_left_motor.data.position = 0;
    handle->back_left_motor.data.target = 0xFF;

    handle->back_right_motor.data.position = 0;
    handle->back_right_motor.data.target = 0xFF;

    Set_Motor_Command(&handle->front_left_motor, &handle->front_right_motor, false, SYNC_VELOCITY_RPM);
    Set_Motor_Command(&handle->front_right_motor, &handle->front_left_motor, true, SYNC_VELOCITY_RPM);

    Set_Motor_Brake(&handle->front_left_motor, GPIO_PIN_RESET);
    Set_Motor_Brake(&handle->front_right_motor, GPIO_PIN_RESET);
   

    Set_Motor_Direction(&handle->front_left_motor, GPIO_PIN_RESET);
    Set_Motor_Direction(&handle->front_right_motor, GPIO_PIN_SET);

    while(true)
    {
        Calculate_Motor_Position(&handle->front_left_motor);
        Calculate_Motor_Position(&handle->front_right_motor);

        if(Is_Velocity_Command_Done(handle, VELOCITY_POSITION_DISTANCE))
        {
            Run_Stop_Command(handle, motor_command);

            Calculate_Motor_Alignment(&handle->front_left_motor);
            Calculate_Motor_Alignment(&handle->front_right_motor);

            break;
        }
        else
        {
            Update_Motor_DAC_PID_Velocity(&handle->front_left_motor);
            Update_Motor_DAC_PID_Velocity(&handle->front_right_motor);

            Update_Multi_DAC(handle, update_indices, 2);
        }
    }
}

void Run_Back_Velocity_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    const float SYNC_VELOCITY_RPM = 500;
    const float VELOCITY_POSITION_DISTANCE = 5.0f;
    uint8_t update_indices[2] = {2, 3};


    handle->front_left_motor.data.position = 0;
    handle->front_left_motor.data.target = 0xFF;

    handle->front_right_motor.data.position = 0;
    handle->front_right_motor.data.target = 0xFF;

    Set_Motor_Command(&handle->back_left_motor, &handle->back_right_motor, false, SYNC_VELOCITY_RPM);
    Set_Motor_Command(&handle->back_right_motor, &handle->back_left_motor, true, SYNC_VELOCITY_RPM);

    Set_Motor_Brake(&handle->back_left_motor, GPIO_PIN_RESET);
    Set_Motor_Brake(&handle->back_right_motor, GPIO_PIN_RESET);
   

    Set_Motor_Direction(&handle->back_left_motor, GPIO_PIN_RESET);
    Set_Motor_Direction(&handle->back_right_motor, GPIO_PIN_SET);

    while(true)
    {
        Calculate_Motor_Position(&handle->back_left_motor);
        Calculate_Motor_Position(&handle->back_right_motor);

        if(Is_Velocity_Command_Done(handle, VELOCITY_POSITION_DISTANCE))
        {
            Run_Stop_Command(handle, motor_command);

            Calculate_Motor_Alignment(&handle->back_left_motor);
            Calculate_Motor_Alignment(&handle->back_right_motor);

            break;
        }
        else
        {
            Update_Motor_DAC_PID_Velocity(&handle->back_left_motor);
            Update_Motor_DAC_PID_Velocity(&handle->back_right_motor);

            Update_Multi_DAC(handle, update_indices, 2);
        }
    }
}

void Run_Left_Velocity_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    const float SYNC_VELOCITY_RPM = 500;
    const float VELOCITY_POSITION_DISTANCE = 5.0f;
    uint8_t update_indices[2] = {0, 2};

    handle->front_right_motor.data.position = 0;
    handle->front_right_motor.data.target = 0xFF;

    handle->back_right_motor.data.position = 0;
    handle->back_right_motor.data.target = 0xFF;

    Set_Motor_Command(&handle->front_left_motor, &handle->back_left_motor, false, SYNC_VELOCITY_RPM);
    Set_Motor_Command(&handle->back_left_motor, &handle->front_left_motor, true, SYNC_VELOCITY_RPM);

    Set_Motor_Brake(&handle->front_left_motor, GPIO_PIN_RESET);
    Set_Motor_Brake(&handle->back_left_motor, GPIO_PIN_RESET);
   

    Set_Motor_Direction(&handle->front_left_motor, GPIO_PIN_RESET);
    Set_Motor_Direction(&handle->back_left_motor, GPIO_PIN_RESET);

    while(true)
    {
        Calculate_Motor_Position(&handle->front_left_motor);
        Calculate_Motor_Position(&handle->back_left_motor);

        if(Is_Velocity_Command_Done(handle, VELOCITY_POSITION_DISTANCE))
        {
            Run_Stop_Command(handle, motor_command);

            Calculate_Motor_Alignment(&handle->front_left_motor);
            Calculate_Motor_Alignment(&handle->back_left_motor);

            break;
        }
        else
        {
            Update_Motor_DAC_PID_Velocity(&handle->front_left_motor);
            Update_Motor_DAC_PID_Velocity(&handle->back_left_motor);

            Update_Multi_DAC(handle, update_indices, 2);
        }
    }
}

void Run_Right_Velocity_Sync(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    const float SYNC_VELOCITY_RPM = 500;
    const float VELOCITY_POSITION_DISTANCE = 5.0f;
    uint8_t update_indices[2] = {1, 3};

    handle->front_left_motor.data.position = 0;
    handle->front_left_motor.data.target = 0xFF;

    handle->back_left_motor.data.position = 0;
    handle->back_left_motor.data.target = 0xFF;

    Set_Motor_Command(&handle->front_right_motor, &handle->back_right_motor, false, SYNC_VELOCITY_RPM);
    Set_Motor_Command(&handle->back_right_motor, &handle->front_right_motor, true, SYNC_VELOCITY_RPM);

    Set_Motor_Brake(&handle->front_right_motor, GPIO_PIN_RESET);
    Set_Motor_Brake(&handle->back_right_motor, GPIO_PIN_RESET);
   
    Set_Motor_Direction(&handle->front_right_motor, GPIO_PIN_RESET);
    Set_Motor_Direction(&handle->back_right_motor, GPIO_PIN_RESET);

    while(true)
    {
        Calculate_Motor_Position(&handle->front_right_motor);
        Calculate_Motor_Position(&handle->back_right_motor);

        if(Is_Velocity_Command_Done(handle, VELOCITY_POSITION_DISTANCE))
        {
            Run_Stop_Command(handle, motor_command);

            Calculate_Motor_Alignment(&handle->front_right_motor);
            Calculate_Motor_Alignment(&handle->back_right_motor);

            break;
        }
        else
        {
            Update_Motor_DAC_PID_Velocity(&handle->front_right_motor);
            Update_Motor_DAC_PID_Velocity(&handle->back_right_motor);

            Update_Multi_DAC(handle, update_indices, 2);
        }
    }
}

void Run_Sync_All_Velocity(Motor_Handle* handle, I2C_Motor_Command* motor_command)
{
    const float SYNC_VELOCITY_RPM = 500;
    const float VELOCITY_POSITION_DISTANCE = 5.0f;

    Set_Motor_Command(&handle->front_left_motor, &handle->front_right_motor, false, SYNC_VELOCITY_RPM);
    Set_Motor_Command(&handle->front_right_motor, &handle->front_left_motor, true, SYNC_VELOCITY_RPM);
    Set_Motor_Command(&handle->back_left_motor, &handle->back_right_motor, false, SYNC_VELOCITY_RPM);
    Set_Motor_Command(&handle->back_right_motor, &handle->back_left_motor, true, SYNC_VELOCITY_RPM);

    for(uint8_t i = 0; i < NUM_MOTORS; i++)
        Set_Motor_Brake(&handle->motors[i], GPIO_PIN_RESET);

    Set_Motor_Direction(&handle->front_left_motor, GPIO_PIN_SET);
    Set_Motor_Direction(&handle->front_right_motor, GPIO_PIN_RESET);

    Set_Motor_Direction(&handle->back_left_motor, GPIO_PIN_SET);
    Set_Motor_Direction(&handle->back_right_motor, GPIO_PIN_RESET);

    while(true)
    {
        for(uint8_t i = 0; i < NUM_MOTORS; i++)
            Calculate_Motor_Position(&handle->motors[i]);


        if(Is_Velocity_Command_Done(handle, VELOCITY_POSITION_DISTANCE))
        {
            Run_Stop_Command(handle, motor_command);

            for(uint8_t i = 0; i < NUM_MOTORS; i++) 
                Calculate_Motor_Alignment(&handle->motors[i]);     
            
            break;
        }
        else
        {
            for(uint8_t i = 0; i < NUM_MOTORS; i++) 
                Update_Motor_DAC_PID_Velocity(&handle->motors[i]);  

            Update_All_DAC(handle);
        }
    }
}



