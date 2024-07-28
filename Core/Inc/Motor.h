#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <assert.h>
#include <stdbool.h>
#include "main.h"


typedef enum 
{
    FORWARD_POSITION_COMMAND = 1u,
    BACKWARDS_POSITION_COMMAND,
    LEFT_TURN_COMMAND,
    RIGHT_TURN_COMMAND,
    STOP_COMMAND,
    NO_COMMAND,
    WRITE_COMMAND,
    READ_COMMAND,
    CRC_COMMAND,
    INVALID_COMMAND
}motor_command_t;



typedef enum
{
    POSITION1 = INVALID_COMMAND + 1u,
    POSITION2,
    POSITION3,
    POSITION4,

    VELOCITY1,
    VELOCITY2,
    VELOCITY3,
    VELOCITY4,

    ALIGNMENT1,
    ALIGNMENT2,
    ALIGNMENT3,
    ALIGNMENT4,
    
    MOTOR_CURRENT_STATE,
    MOTOR_NEXT_STATE,

    PID_KP1,
    PID_KI1,
    PID_KD1,
    PID_MIN1,
    PID_MAX1,

    PID_KP2,
    PID_KI2,
    PID_KD2,
    PID_MIN2,
    PID_MAX2,

    PID_KP3,
    PID_KI3,
    PID_KD3,
    PID_MIN3,
    PID_MAX3,

    PID_KP4,
    PID_KI4,
    PID_KD4,
    PID_MIN4,
    PID_MAX4,

    TARGET_POSITON,
    TARGET_VELOCITY,

    INVALID_VALUE
}motor_value_t;





typedef struct
{
    GPIO_TypeDef* type;
    uint16_t pin;
    GPIO_PinState state;
}GPIO_Pin;


typedef struct
{
    uint8_t channel;
    uint16_t dac_value;
}DAC_Pin;


typedef struct
{
	DAC_Pin dac;
    GPIO_Pin pin_u;
    GPIO_Pin pin_v;
    GPIO_Pin pin_w;
    GPIO_Pin* direction;
    GPIO_Pin* brake;
    GPIO_Pin* enable;
}Motor_Pins; 


typedef struct
{
    float step_time[16];
    volatile float avg_step_sum;
    float revolutions;
    float updated_position;
    float position;
    float target;
    float velocity;
    float alignment;
    uint32_t last_time;
    uint32_t last_steps;
    volatile uint8_t rpm_index;
    volatile bool step_buffer_filled;
}Motor_Data; 


typedef struct 
{
    const float kp;
    const float ki;
    const float kd;
    const float min;
    const float max;
    const uint16_t dac_default;
}PID_Parameters;


typedef struct
{
    float delta_t; 

    float e_integral;
    float e_proportional;
    float e_derivative;

    float e_prev;

    uint32_t steps;
    uint32_t errors;
    uint32_t last_time_pid;
    int8_t direction;

    int8_t is_saturated;
    uint8_t state;
}Hall_State;

typedef struct Motor Motor;

struct Motor
{
	Motor_Pins pins;

	Motor_Data data;

	PID_Parameters pid_position;

	PID_Parameters pid_velocity;

	Hall_State state;

	Motor *relative;

	bool is_child;
};



/**
  * @brief Reads the GPIO
  * @param GPIO GPIO_Pin structure
  * @return GPIO_PinState 
  */
GPIO_PinState GPIO_Read(GPIO_Pin* GPIO);


/**
  * @brief Writes to the GPIO
  * @param GPIO GPIO_Pin structure
  * @param state State to write
  */
void GPIO_Write(GPIO_Pin* GPIO, const GPIO_PinState state);

/**
  * @brief Initializes a GPIO as output 
  * @param GPIO GPIO_Pin structure
  * @param type Pointer to GPIO_TypeDef 
  * @param pin Pin Number
  * @param default_state Default Pin State
  */
void GPIO_Output_Init(GPIO_Pin* GPIO, GPIO_TypeDef* type, const uint16_t pin, const GPIO_PinState default_state);
/**
  * @brief Initialies a GPIO as input
  * @param GPIO GPIO_Pin structure
  * @param type Pointer to GPIO_TypeDef
  * @param pin Pin Number
  */
void GPIO_Input_Init(GPIO_Pin* GPIO, GPIO_TypeDef* type, const uint16_t pin);

/**
  * @brief Sets the PWM Duty Cycle
  * @param PWM PWM_Pin structure
  * @param duty_cycle Duty Cycle
  */
void DAC_Set_Value(DAC_Pin* pin, uint16_t dac_value);
/**
  * @brief Initializes DAC
  * @param pin Pointer to DAC_Pin structure
  * @param channel DAC channel
  * @param default_dac default value for DAC
  */
void DAC_Init(DAC_Pin* pin, uint8_t channel, uint16_t default_dac);

/**
  * @brief Copies PID Parameters to motor
  * @param motor Pointer to Motor
  * @param parameters Pointer to PID Paramters
  */
void Init_Motor_Pid_Position(Motor* motor, const PID_Parameters* parameters);

/**
  * @brief Copies PID Parameters to motor
  * @param motor Pointer to Motor
  * @param parameters Pointer to PID Paramters
  */
void Init_Motor_Pid_Velocity(Motor* motor, const PID_Parameters* parameters);

/**
  * @brief Updates Motor Hall State with the Hall GPIOs
  * @param motor Pointer to Motor
  * @return The Current Hall State
  */
uint8_t Update_Hall_State(Motor* motor);


/// @brief Sets the Direction GPIO
/// @param motor Pointer to Motor
/// @param state Motor direction
void Set_Motor_Direction(Motor* motor, const GPIO_PinState state);


/// @brief Sets the Motor Brake GPIO
/// @param motor Pointer to Motor
/// @param state Motor Brake State
void Set_Motor_Brake(Motor* motor, const GPIO_PinState state);

/// @brief Resets the Motor controlle 
/// @param motor Pointer to Motor
void Reset_Motor(Motor* motor);


/// @brief Sets the PWM value to ZERO and sets brake pin to HIGH
/// @param motor Pointer to Motor
void Stop_Motor(Motor* motor);


/// @brief Initializes the Motor Pins
/// @param dac_channel motor dac channel
/// @param default_dac motor dac default value
/// @param motor Pointer to Motor
/// @param U_Type GPIO_TypeDef U_Hall pointer
/// @param U_Pin U Pin Number
/// @param V_Type GPIO_TypeDef V_Hall pointer
/// @param V_Pin V Pin Number
/// @param W_Type GPIO_TypeDef W_Hall Pointer
/// @param W_Pin W Pin Number 
/// @param tim TIM_HandleTypeDef pointer
/// @param channel Tim Chnanel
/// @param direction GPIO_Pin direction pointer 
/// @param brake GPIO_Pin brake pointer
/// @param enable GPIO_Pin enable pointer
void Init_Motor_Pins(Motor* motor, 
                    const uint8_t dac_channel, const uint16_t default_dac,
                    GPIO_TypeDef* U_Type, const uint16_t U_Pin, 
                    GPIO_TypeDef* V_Type, const uint16_t V_Pin, 
                    GPIO_TypeDef* W_Type, const uint16_t W_Pin, 
                    GPIO_Pin* direction, GPIO_Pin* brake, GPIO_Pin* enable);



/// @brief Sets Motor Relative
/// @param motor Pointer to Motor
/// @param relative Relative Motor
/// @param is_child Is A Child
void Set_Motor_Relative(Motor* motor, Motor* relative, bool is_child);


/// @brief Calculates the Motor Position
/// @param motor Pointer to Motor
void Calculate_Motor_Position(Motor* motor);


/// @brief Calculates the Motor Alignment
/// @param motor Pointer to Motor
void Calculate_Motor_Alignment(Motor* motor);


/// @brief Updates Motor Position
/// @param motor Pointer to Motor
/// @param updated_position Updated Position
void Update_Motor_Position(Motor* motor, const float updated_position);


/// @brief Periodic call to update Motor Hall State
/// @param motor Pointer to motor
/// @param tim Timer Pointer used for callback
void Update_Motor_Period_Elapsed(Motor* motor, TIM_HandleTypeDef* tim);


/// @brief Updates motor DAC value with PID for Position
/// @param motor Pointer to Motor
void Update_Motor_DAC_PID_Position(Motor* motor);

/// @brief Updates motor DAC value with PID for Velocity
/// @param motor Pointer to Motor
void Update_Motor_DAC_PID_Velocity(Motor* motor);

/// @brief Provides the motor data into the given buffer
/// @param buffer to write
/// @param buffer_size size of buffer
/// @return Number of bytes written
uint8_t Get_Motor_Data(Motor* motor, uint8_t* buffer, const uint8_t buffer_size, const uint8_t motor_index);

/// @brief Returns if a given byte is considered a valid Command
/// @param byte Byte
/// @return If valid
bool Is_Valid_Motor_Command(const uint8_t byte);

/// @brief Returns payload size of a given command
/// @param command Byte
/// @return Command payload size
uint8_t Get_Command_Size(const uint8_t command);


/// @brief Returns if a given byte is considered a valid Value
/// @param byte Byte
/// @return If valid
bool Is_Valid_Value(const uint8_t byte);


/// @brief Returns payload size of a given value
/// @param value byte
/// @return Value payload size
uint8_t Get_Value_Size(const uint8_t value);


/// @brief Returns if the value is writable
/// @param value byte
/// @return If value is writable
bool Is_Writable_Value(const uint8_t value);


/// @brief Returns the motor index for the value
/// @param value byte
/// @return 1 == PID1, 2 == PID2, 3 == PID3, 4 == PID4, 5 == TARGET_POSITION, 6 == TARGET_VELOCITY
uint8_t Get_Motor_Value_ID(const uint8_t value);


/// @brief Sets the next motor command
/// @param motor Pointer to Motor
/// @param relative Pointer to relative
/// @param is_child is_child to relative
/// @param target target position or velocity
/// @param state state of the motor
void Set_Motor_Command(Motor* motor, Motor* relative, const bool is_child, const float target);



void Init_Motor_Timer();

#endif // MOTOR_H

















