#include "Motor.h"
#include <string.h>
#include <math.h>

#define PI 3.1415926535897932384
#define INCH_TO_METERS 0.02540005    // Converting inch to meters
#define WHEEL_DIAM_INCH 18           // Wheel diameter inch
#define GEARBOX_REDUCTION_RATIO 30.0 // gearbox reduction ratio

#define MAX_HALL_STEPS 2
#define STEPS_PER_ROTATION 18

#define HALL_SPEED_FIFO_SIZE 18
#define MOTOR_DIR_POSITIVE  (int8_t)1
#define MOTOR_DIR_NEGATIVE  (int8_t)-1


const float HALL_STEP_CONSTANT = 0.05555555556;                            // 18 steps per rotation
const float GEARBOX_RATIO = 1.0 / GEARBOX_REDUCTION_RATIO;                 // Gear box reduction ratio
const float WHEEL_CIRCUMFERENCE = (WHEEL_DIAM_INCH * INCH_TO_METERS) * PI; // Wheel circumference in meters
const uint32_t RPM_TIME_PERIOD = 100;
TIM_HandleTypeDef* TIM_INSTANCE;


#define HALL_STATE_0 (uint8_t)0
#define HALL_STATE_1 (uint8_t)1
#define HALL_STATE_2 (uint8_t)2
#define HALL_STATE_3 (uint8_t)3
#define HALL_STATE_4 (uint8_t)4
#define HALL_STATE_5 (uint8_t)5
#define HALL_STATE_6 (uint8_t)6
#define HALL_STATE_7 (uint8_t)7


#define ABS(x)  (x<0)?-x:x

const float MILLISECONDS_TO_SECONDS = 0.001;                               // Milliseconds to seconds






uint32_t Get_Micros()
{
    const uint32_t time = TIM2->CNT;
    return time;
}



GPIO_PinState GPIO_Read(GPIO_Pin* GPIO)
{
#if DEBUG_MODE
    if(GPIO == NULL || GPIO->type == NULL)
        Error_Handler();
#endif

    GPIO->state = HAL_GPIO_ReadPin(GPIO->type, GPIO->pin);
    return GPIO->state;
}

void GPIO_Write(GPIO_Pin* GPIO, const GPIO_PinState state)
{
#if DEBUG_MODE
    if(GPIO == NULL || GPIO->type == NULL)
        Error_Handler();
#endif

    if(state != GPIO->state)
    {
        GPIO->state = state;
        HAL_GPIO_WritePin(GPIO->type, GPIO->pin, GPIO->state);
    }
}

void GPIO_Output_Init(GPIO_Pin* GPIO, GPIO_TypeDef* type, const uint16_t pin, const GPIO_PinState default_state)
{
#if DEBUG_MODE
    if(GPIO == NULL || type == NULL)
        Error_Handler();
#endif

    GPIO->type = type; 
    GPIO->pin = pin;
    GPIO->state = default_state;
    HAL_GPIO_WritePin(GPIO->type, GPIO->pin, GPIO->state);
}

void GPIO_Input_Init(GPIO_Pin* GPIO, GPIO_TypeDef* type, const uint16_t pin)
{
#if DEBUG_MODE
    if(GPIO == NULL || type == NULL)
        Error_Handler();
#endif 

    GPIO->type = type; 
    GPIO->pin = pin;
    GPIO->state = HAL_GPIO_ReadPin(GPIO->type, GPIO->pin);
}



void DAC_Set_Value(DAC_Pin* pin, uint16_t dac_value)
{
	if(dac_value > 4095)
		dac_value = 4095;
	pin->dac_value = dac_value;
}


void DAC_Init(DAC_Pin* pin, uint8_t channel, uint16_t default_dac)
{
#if DEBUG_MODE
	if(pin == NULL)
		Error_Handler();
#endif
	pin->channel = channel;
	DAC_Set_Value(pin, default_dac);
}




void Reset_Motor(Motor* motor)
{
#if DEBUG_MODE
    if(motor == NULL)
        Error_Handler();
#endif

    const uint8_t hstate = motor->state.state;

    memset(&motor->data, 0, sizeof(Motor_Data));
    memset(&motor->state, 0, sizeof(Hall_State));

    motor->state.state = hstate;
}


void Set_Motor_Brake(Motor* motor, const GPIO_PinState state)
{
#if DEBUG_MODE
    if(motor == NULL)
        Error_Handler();
#endif

    GPIO_Write(motor->pins.brake, state);
}


void Stop_Motor(Motor* motor)
{
#if DEBUG_MODE
    if(motor == NULL)
        Error_Handler();
#endif 


    // PWM_Set_Duty_Cycle(&motor->pins.pwm, 0);
    //Set_Motor_Brake(motor, GPIO_PIN_SET);
}



bool is_valid_hall_state(const uint8_t state)
{
	switch(state)
	{
	case HALL_STATE_1:
	case HALL_STATE_2:
	case HALL_STATE_3:
	case HALL_STATE_4:
	case HALL_STATE_5:
	case HALL_STATE_6:
		return true;
	default:
		return false;
	}
}


void Init_Motor_Pins(Motor* motor, 
                     const uint8_t dac_channel, const uint16_t default_dac, 
                     GPIO_TypeDef* U_Type, const uint16_t U_Pin, 
                     GPIO_TypeDef* V_Type, const uint16_t V_Pin, 
                     GPIO_TypeDef* W_Type, const uint16_t W_Pin,
                     GPIO_Pin* direction, GPIO_Pin* brake, GPIO_Pin* enable)
{
#if DEBUG_MODE
    if(motor == NULL || direction == NULL || brake == NULL)
        Error_Handler();
#endif

    Reset_Motor(motor);

    DAC_Init(&motor->pins.dac, dac_channel, default_dac);

    GPIO_Input_Init(&motor->pins.pin_u, U_Type, U_Pin);
    GPIO_Input_Init(&motor->pins.pin_v, V_Type, V_Pin);
    GPIO_Input_Init(&motor->pins.pin_w, W_Type, W_Pin);

    //PWM_Init(&motor->pins.pwm, tim, channel);

    motor->pins.direction = direction;
    motor->pins.brake = brake;
    motor->state.state  = (uint8_t)((motor->pins.pin_w.state << 2) | (motor->pins.pin_v.state << 1) | motor->pins.pin_u.state );

    if(!is_valid_hall_state(motor->state.state))
    	Error_Handler();
}

void Init_Motor_Pid_Position(Motor* motor, const PID_Parameters* parameters)
{
#if DEBUG_MODE
    if(parameters == NULL)
        Error_Handler();
#endif

    memcpy(&motor->pid_position, parameters, sizeof(PID_Parameters));
}

void Init_Motor_Pid_Velocity(Motor* motor, const PID_Parameters* parameters)
{
#if DEBUG_MODE
    if(parameters == NULL)
        Error_Handler();
#endif

    memcpy(&motor->pid_velocity, parameters, sizeof(PID_Parameters));
}

uint8_t Get_Hall_State(Motor* motor)
{
#if DEBUG_MODE
    if(motor == NULL)
        Error_Handler();
#endif

    const uint8_t state = (uint8_t)((GPIO_Read(&motor->pins.pin_w) << 2) | (GPIO_Read(&motor->pins.pin_v) << 1) | GPIO_Read(&motor->pins.pin_u));
    return state;
}


void Set_Motor_Direction(Motor* motor, const GPIO_PinState state)
{
#if DEBUG_MODE
    if(motor == NULL)
        Error_Handler();
#endif

    GPIO_Write(motor->pins.direction, state);
}


void Set_Motor_Relative(Motor* motor, Motor* relative, bool is_child)
{
#if DEBUG_MODE
    if(motor == NULL || relative == NULL)
        Error_Handler();
#endif

    motor->relative = relative;
    motor->is_child = is_child;

    relative->relative = motor;
    relative->is_child = !is_child;
}

void Calculate_Motor_Position(Motor* motor)
{
#if DEBUG_MODE
    if(motor == NULL)
        Error_Handler();
#endif
    motor->data.revolutions = (float)motor->state.steps * HALL_STEP_CONSTANT * GEARBOX_RATIO;
    motor->data.position = (motor->data.revolutions * WHEEL_CIRCUMFERENCE) + motor->data.updated_position;


    // Not super safe, each of these variables are single cycle but could be read at different times
    const bool step_buffer_filled = motor->data.step_buffer_filled;
    const uint8_t rpm_index = motor->data.rpm_index;
    const float avg_step_sum = motor->data.avg_step_sum;


    if(rpm_index != 0 || step_buffer_filled)
    {
        const float denom = step_buffer_filled ? 16.0f : rpm_index; 
        const float avg_step = avg_step_sum / denom;                // compute the average step time in seconds
        const float steps_per_min = 60.0 / avg_step;                // compute the number of steps per minute
        motor->data.velocity = steps_per_min * HALL_STEP_CONSTANT;  // convert hall signal steps to RPMs
    }
    else
        motor->data.velocity = 0; // undetectable so far
    


    
   




    // if(motor->data.last_time == 0)
    // {
    //     motor->data.last_time = HAL_GetTick();
    //     motor->data.last_steps = motor->state.steps;
    // }
    // else
    // {
    //     const uint32_t num_steps = motor->state.steps - motor->data.last_steps;
    //     if(num_steps != 0)
    //     {
    //         const uint32_t now = HAL_GetTick();
    //         const uint32_t elapsed_time = now - motor->data.last_time;
    //         motor->data.last_time = now;
    //         motor->data.last_steps = motor->state.steps;

    //         const float revolutions = (float)num_steps * HALL_STEP_CONSTANT;
    //         //motor->data.rpms = revolutions / ((float)elapsed_time * MILLISECONDS_TO_SECONDS);
    //         motor->data.velocity = revolutions * ((float)elapsed_time * MILLISECONDS_TO_SECONDS);
    //     }

    //     // once way
    //     // const uint32_t steps_taken = motor->state.steps - motor->data.last_steps;
    //     // if(steps_taken > STEPS_PER_ROTATION)
    //     // {
    //     //     motor->data.last_steps = motor->state.steps;
    //     //     const uint32_t now = HAL_GetTick();
    //     //     const uint32_t elapsed = now - motor->data.last_time; // time taken for rotation
    //     //     motor->data.last_time = now;
    //     //     motor->data.velocity = 60.0 / ((float)elapsed * MILLISECONDS_TO_SECONDS);
    //     // }

    //     // const uint32_t now = HAL_GetTick();
    //     // const uint32_t time_elapsed = now - motor->data.last_time;
    //     // if(time_elapsed > 50) // 50 ms
    //     // {
    //     //     const uint32_t steps_taken = motor->state.steps - motor->data.last_steps;
    //     //     const float revolutions = (float)steps_taken * HALL_STEP_CONSTANT;
    //     //     motor->data.velocity = revolutions * ((float)time_elapsed * MILLISECONDS_TO_SECONDS);

    //     //     motor->data.last_time = now;
    //     //     motor->data.last_steps = motor->state.steps;
    //     // }
    // }
}


void Calculate_Motor_Alignment(Motor* motor)
{
#if DEBUG_MODE
    if(motor == NULL || motor->relative == NULL)
        Error_Handler();
#endif
    const float difference = motor->data.position - motor->relative->data.position;
    motor->data.alignment = ABS(difference);
}

void Update_Motor_Position(Motor* motor, const float updated_position)
{
#if DEBUG_MODE
    if(motor == NULL)
        Error_Handler();
#endif

    motor->state.steps = 0;
    motor->data.updated_position = updated_position;
}


void Update_Motor_Period_Elapsed(Motor* motor, TIM_HandleTypeDef* tim)
{
#if DEBUG_MODE
    if(motor == NULL)
        Error_Handler();
#endif
    if(!is_valid_hall_state(motor->state.state))
        Error_Handler();
    
    const uint8_t next_hall_state = Get_Hall_State(motor);

    if(next_hall_state != motor->state.state)
    {

        if(motor->data.last_time == 0)
        {
            motor->data.last_time = Get_Micros();
        }
        else
        {
            const uint32_t now = Get_Micros();

            motor->data.step_time[motor->data.rpm_index] = (1.0 / (double)SystemCoreClock) * (double)(now - motor->data.last_time);
            motor->data.avg_step_sum += motor->data.step_time[motor->data.rpm_index];
            motor->data.last_time = now;

            motor->data.rpm_index++;
            if(motor->data.rpm_index == 16)
            {
                motor->data.step_buffer_filled = true;
                motor->data.rpm_index = 0;
            }
            if(motor->data.step_buffer_filled)
            {
                motor->data.avg_step_sum -= motor->data.step_time[motor->data.rpm_index]; // remove value from the accumlator
            }
        }

        switch(next_hall_state)
        {
        case HALL_STATE_1:
            if(motor->state.state == HALL_STATE_5)
            {
                motor->state.steps++;
                motor->state.state = next_hall_state;
                motor->state.direction = MOTOR_DIR_POSITIVE;
            }
            else if(motor->state.state == HALL_STATE_3)
            {
                motor->state.steps++;
                motor->state.state = next_hall_state;
                motor->state.direction = MOTOR_DIR_NEGATIVE;
            }
            else
            {
                motor->state.errors++;
                motor->state.state = next_hall_state;
            }
            break;
        case HALL_STATE_2:
            if(motor->state.state == HALL_STATE_3) // positive
			{
				motor->state.steps++;
                motor->state.state = next_hall_state;
                motor->state.direction = MOTOR_DIR_POSITIVE;
			}
			else if(motor->state.state == HALL_STATE_6) // negative
			{
				motor->state.steps++;
                motor->state.state = next_hall_state;
                motor->state.direction = MOTOR_DIR_NEGATIVE;
			}
			else
			{
				motor->state.errors++;
                motor->state.state = next_hall_state;
			}
            break;
        case HALL_STATE_3:
            if(motor->state.state == HALL_STATE_1) // positive
			{
				motor->state.steps++;
                motor->state.state = next_hall_state;
                motor->state.direction = MOTOR_DIR_POSITIVE;
			}
			else if(motor->state.state == HALL_STATE_2) // negative
			{
				motor->state.steps++;
                motor->state.state = next_hall_state;
                motor->state.direction = MOTOR_DIR_NEGATIVE;
			}
			else
			{
				motor->state.errors++;
                motor->state.state = next_hall_state;
			}
            break;
        case HALL_STATE_4:
            if(motor->state.state == HALL_STATE_6) // positive
			{
				motor->state.steps++;
                motor->state.state = next_hall_state;
                motor->state.direction = MOTOR_DIR_POSITIVE;
			}
			else if(motor->state.state == HALL_STATE_5) // negative
			{
				motor->state.steps++;
                motor->state.state = next_hall_state;
                motor->state.direction = MOTOR_DIR_NEGATIVE;
			}
			else
			{
				motor->state.errors++;
                motor->state.state = next_hall_state;
			}
            break;
        case HALL_STATE_5:
            if(motor->state.state == HALL_STATE_4) // positive
			{
				motor->state.steps++;
                motor->state.state = next_hall_state;
                motor->state.direction = MOTOR_DIR_POSITIVE;
			}
			else if(motor->state.state == HALL_STATE_1) // negative
			{
				motor->state.steps++;
                motor->state.state = next_hall_state;
                motor->state.direction = MOTOR_DIR_NEGATIVE;
			}
			else
			{
				motor->state.errors++;
                motor->state.state = next_hall_state;
			}
            break;
        case HALL_STATE_6:
            if(motor->state.state == HALL_STATE_2) // positive
			{
				motor->state.steps++;
                motor->state.state = next_hall_state;
                motor->state.direction = MOTOR_DIR_POSITIVE;
			}
			else if(motor->state.state == HALL_STATE_4) // negative
			{
				motor->state.steps++;
                motor->state.state = next_hall_state;
                motor->state.direction = MOTOR_DIR_NEGATIVE;
			}
			else
			{
				motor->state.errors++;
                motor->state.state = next_hall_state;
			}
            break;
        default:
            motor->state.errors++;
            break;
        }
    }
    
}



int8_t clamp_pid(float* value, const float min, const float max)
{
	if(*value < min)
	{
		*value = min;
		return -1;
	}
	if(*value > max)
	{
		*value = max;
		return 1;
	}
	return 0;
}




void Update_Motor_DAC_PID_Position(Motor* motor)
{
#if DEBUG_MODE
    if(motor == NULL || motor->relative == NULL || (!motor->is_child && !motor->relative->is_child))
        Error_Handler();
#endif
    float output, error;
    const float position = motor->data.position;
    const float relative_position = motor->relative->data.position;

    const uint32_t now = Get_Micros();
    motor->state.delta_t = (1.0 / (double)SystemCoreClock) * (double)(now - motor->state.last_time_pid); 
    motor->state.last_time_pid = now;
   
    if(motor->state.delta_t <= 0 ||  motor->state.delta_t > 0.5f)
        motor->state.delta_t = 1e-3f;

    // Cross-Coupling Method
    if(motor->is_child)
        error = motor->data.target + relative_position - (2.0f * position);
    else
        error = motor->data.target - (2.0f * position) + relative_position;

    // Calculate the proportional
    motor->state.e_proportional = motor->pid_position.kp * error;

    // Calculate the integral
    bool freeze_integral = (motor->state.is_saturated && motor->state.is_saturated * error > 0);

    if(!freeze_integral) // prevent integral wind up
    	motor->state.e_integral += motor->pid_position.ki * error * motor->state.delta_t;

    clamp_pid(&motor->state.e_integral, motor->pid_position.min, motor->pid_position.max);

    // Calculate the derivative
    if(isnan(motor->state.e_prev)) // first run
        motor->state.e_prev = error;
    
    motor->state.e_derivative = motor->pid_position.kd * ((error - motor->state.e_prev) / motor->state.delta_t);

    output = motor->state.e_proportional + motor->state.e_integral + motor->state.e_derivative;

    motor->state.is_saturated = clamp_pid(&output, motor->pid_position.min, motor->pid_position.max);

    motor->state.e_prev = error;

    motor->pins.dac.dac_value = (uint16_t)(output * 0.01f * (4095 - motor->pid_position.dac_default)) + motor->pid_position.dac_default;
}






void Update_Motor_DAC_PID_Velocity(Motor* motor)
{
#if DEBUG_MODE
    if(motor == NULL || motor->relative == NULL || (!motor->is_child && !motor->relative->is_child))
        Error_Handler();
#endif
    const uint32_t now = Get_Micros();

    float output, error;
    float velocity = motor->data.velocity;
    float relative_velocity = motor->relative->data.velocity;

    motor->state.delta_t = (1.0 / (double)SystemCoreClock) * (double)(now - motor->state.last_time_pid); 
    motor->state.last_time_pid = now;
   
    if(motor->state.delta_t <= 0 ||  motor->state.delta_t > 0.5f)
        motor->state.delta_t = 1e-3f;


    if(motor->is_child)
        error = motor->data.target + velocity - (2.0 * velocity);
    else
        error = motor->data.target - (2.0 * velocity) + relative_velocity;


    motor->state.e_proportional = motor->pid_velocity.kp * error;


    if(motor->state.is_saturated != 1) // prevent integral wind up
    	motor->state.e_integral = motor->state.e_integral + motor->pid_velocity.ki * motor->state.delta_t * 0.5f *(error + motor->state.e_prev);

    clamp_pid(&motor->state.e_integral, motor->pid_velocity.min, motor->pid_velocity.max);

    motor->state.e_derivative = motor->pid_velocity.kd * (error - motor->state.e_prev) / motor->state.delta_t;

    output = motor->state.e_proportional + motor->state.e_integral + motor->state.e_derivative;

    motor->state.is_saturated = clamp_pid(&output, motor->pid_velocity.min, motor->pid_velocity.max);

    motor->state.e_prev = error;

    motor->pins.dac.dac_value = (uint16_t)output;
}



uint8_t Get_Motor_Position_Value(const uint8_t index)
{
    switch(index)
    {
        case 0:
            return POSITION1;
        case 1:
            return POSITION2;
        case 2:
            return POSITION3;
        case 3:
            return POSITION4;
        default:
        	Error_Handler();
        	return 0;
    }
}

uint8_t Get_Motor_Velocity_Value(const uint8_t index)
{
    switch(index)
    {
        case 0:
            return VELOCITY1;
        case 1:
            return VELOCITY2;
        case 2:
            return VELOCITY3;
        case 3:
            return VELOCITY4;
        default:
        	Error_Handler();
        	return 0;
    }
}

uint8_t Get_Motor_Alignment_Value(const uint8_t index)
{
    switch(index)
    {
        case 0:
            return ALIGNMENT1;
        case 1:
            return ALIGNMENT2;
        case 2:
            return ALIGNMENT3;
        case 3:
            return ALIGNMENT4;
        default:
        	Error_Handler();
        	return 0;
    }
}   






uint8_t Get_Motor_Data(Motor* motor, uint8_t* buffer, const uint8_t buffer_size, const uint8_t motor_index)
{
#if DEBUG_MODE
    if(sizeof(float) * 4  + sizeof(uint8_t) * 4 > buffer_size || motor == NULL || buffer == NULL)
        Error_Handler();
#endif

    const uint8_t position_value = Get_Motor_Position_Value(motor_index);
    const uint8_t velocity_value = Get_Motor_Velocity_Value(motor_index);
    const uint8_t alignment_value = Get_Motor_Alignment_Value(motor_index);


    uint8_t offset = 0;

    buffer[offset] = position_value;
    offset++;
    memcpy(buffer + offset, &motor->data.position, sizeof(float));
    offset += sizeof(float);

    buffer[offset] = velocity_value;
    offset++;
    memcpy(buffer + offset, &motor->data.velocity, sizeof(float));
    offset += sizeof(float); 

    buffer[offset] = alignment_value;
    offset++; 
    memcpy(buffer + offset, &motor->data.alignment, sizeof(float));
    offset += sizeof(float);

    return offset;
}

bool Is_Valid_Motor_Command(const uint8_t byte)
{
    switch(byte)
    {
        case FORWARD_POSITION_COMMAND:
        case BACKWARDS_POSITION_COMMAND:
        case LEFT_TURN_COMMAND:
        case RIGHT_TURN_COMMAND:
        case STOP_COMMAND:
            return true;
        default:
            return false;
    }
}

uint8_t Get_Command_Size(const uint8_t command)
{
    switch(command)
    {
        case FORWARD_POSITION_COMMAND:
        case BACKWARDS_POSITION_COMMAND:
            return sizeof(float);
        case LEFT_TURN_COMMAND:
        case RIGHT_TURN_COMMAND:
        case STOP_COMMAND:  
            return 0;
        default:
            Error_Handler();
            return 0;
    }
}



bool Is_Valid_Value(const uint8_t byte)
{
    switch(byte)
    {
        case PID_KP1:
        case PID_KI1:
        case PID_KD1:
        case PID_MIN1:
        case PID_MAX1:

        case PID_KP2:
        case PID_KI2:
        case PID_KD2:
        case PID_MIN2:
        case PID_MAX2:

        case PID_KP3:
        case PID_KI3:
        case PID_KD3:
        case PID_MIN3:
        case PID_MAX3:

        case PID_KP4:
        case PID_KI4:
        case PID_KD4:
        case PID_MIN4:
        case PID_MAX4:

        case TARGET_POSITON:
        case TARGET_VELOCITY:
            return true;
        default:
            return false;
    }
}



uint8_t Get_Value_Size(const uint8_t value)
{
    switch(value)
    {
        case PID_KP1:
        case PID_KI1:
        case PID_KD1:
        case PID_MIN1:
        case PID_MAX1:

        case PID_KP2:
        case PID_KI2:
        case PID_KD2:
        case PID_MIN2:
        case PID_MAX2:

        case PID_KP3:
        case PID_KI3:
        case PID_KD3:
        case PID_MIN3:
        case PID_MAX3:

        case PID_KP4:
        case PID_KI4:
        case PID_KD4:
        case PID_MIN4:
        case PID_MAX4:

        case TARGET_POSITON:
        case TARGET_VELOCITY:
            return sizeof(float);
        default:
            return 0;
    }
}


bool Is_Writable_Value(const uint8_t value)
{
    switch(value)
    {
        case PID_KP1:
        case PID_KI1:
        case PID_KD1:
        case PID_MIN1:
        case PID_MAX1:

        case PID_KP2:
        case PID_KI2:
        case PID_KD2:
        case PID_MIN2:
        case PID_MAX2:

        case PID_KP3:
        case PID_KI3:
        case PID_KD3:
        case PID_MIN3:
        case PID_MAX3:

        case PID_KP4:
        case PID_KI4:
        case PID_KD4:
        case PID_MIN4:
        case PID_MAX4:

        case TARGET_POSITON:
        case TARGET_VELOCITY:
            return true;
        default:
            return false;
    }
}



uint8_t Get_Motor_Value_ID(const uint8_t value)
{
    switch(value)
    {
        case PID_KP1:
        case PID_KI1:
        case PID_KD1:
        case PID_MIN1:
        case PID_MAX1:
            return 1;
        case PID_KP2:
        case PID_KI2:
        case PID_KD2:
        case PID_MIN2:
        case PID_MAX2:
            return 2;
        case PID_KP3:
        case PID_KI3:
        case PID_KD3:
        case PID_MIN3:
        case PID_MAX3:
            return 3;
        case PID_KP4:
        case PID_KI4:
        case PID_KD4:
        case PID_MIN4:
        case PID_MAX4:
            return 4;
        case TARGET_POSITON:
            return 5;
        case TARGET_VELOCITY:
            return 6;
        default:
            return 0;
    }
}


void Set_Motor_Command(Motor* motor, Motor* relative, const bool is_child, const float target)
{
#if DEBUG_MODE
    if(motor == NULL || relative == NULL)
        Error_Handler();
#endif

    Reset_Motor(motor);
    motor->state.last_time_pid = Get_Micros();
    motor->state.e_prev = NAN;
    motor->relative = relative;
    motor->is_child = is_child;
    motor->data.target = target;
}




void Init_Motor_Timer()
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->PSC = HAL_RCC_GetPCLK1Freq() / 1000000 - 1;
    TIM2->CR1 = TIM_CR1_CEN;
}

