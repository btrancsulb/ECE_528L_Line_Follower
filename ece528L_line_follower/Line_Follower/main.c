/**
 * @file main.c
 *
 * @brief Main source code for the Line_Follower program.
 *
 * This file contains the main entry point for the Line_Follower program.
 * The main controller demonstrates a Line Follower robot without using an advanced algorithm.
 *
 * It interfaces the following peripherals using GPIO to demonstrate line following:
 *  - 8-Channel QTRX Sensor Array module
 *
 * Timers are used in this lab:
 *  - SysTick:  Used to generate periodic interrupts at a specified rate (1 kHz)
 *  - Timer A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (1 kHz)
 *
 * @note For more information regarding the 8-Channel QTRX Sensor Array module,
 * refer to the product page: https://www.pololu.com/product/3672
 *
 * @author
 *
 */

#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"

// Initializing constant PWM duty cycle values for the motors
#define PWM_NOMINAL 2500U
#define PWM_SLOW 1500U

// Initialize a global variable for Timer A1 to keep track of elapsed time in milliseconds
static uint32_t Timer_A1_ms_elapsed = 0;
static volatile uint32_t system_ms = 0;

// Bit 0 is the rightmost sensor; bit 7 is the leftmost sensor.
typedef enum {
    CENTER              = 0x18,
    SLIGHT_LEFT_CENTER  = 0x38,
    SLIGHT_RIGHT_CENTER = 0x1C,
    FAR_LEFT            = 0x80,
    FAR_RIGHT           = 0x01,
    LEFT_TURN           = 0xF8,
    RIGHT_TURN          = 0x1F,
    T_INTERSECTION      = 0xFF,
    DEAD_END            = 0x00
} Line_Position;

static void Follow_Line(uint8_t data);

void Detect_Line_Position(uint8_t reflectance_sensor_data)
{
    LED1_Output(RED_LED_OFF);
    switch (reflectance_sensor_data) {
    case CENTER:
        LED2_Output(RGB_LED_GREEN);
        break;
    case SLIGHT_LEFT_CENTER:
        LED2_Output(RGB_LED_YELLOW);
        break;
    case SLIGHT_RIGHT_CENTER:
        LED2_Output(RGB_LED_PINK);
        break;
    case FAR_LEFT:
        LED2_Output(RGB_LED_WHITE);
        break;
    case FAR_RIGHT:
        LED2_Output(RGB_LED_SKY_BLUE);
        break;
    case LEFT_TURN:
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_WHITE);
        Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
        return;
    case RIGHT_TURN:
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_SKY_BLUE);
        Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
        return;
    case T_INTERSECTION:
        LED2_Output(RGB_LED_BLUE);
        Motor_Stop();
        return;
    case DEAD_END:
        LED2_Output(RGB_LED_RED);
        Motor_Stop();
        return;
    default:
        LED2_Output(RGB_LED_OFF);
        break;
    }

    Follow_Line(reflectance_sensor_data);
}

void Timer_A1_Periodic_Task(void)
{
    // Start at 10, 20, 30, ... ms and finish one tick later.
    Timer_A1_ms_elapsed++;
    if (Timer_A1_ms_elapsed == 10U) {
        Reflectance_Sensor_Start();
    } else if (Timer_A1_ms_elapsed == 11U) {
        uint8_t data = Reflectance_Sensor_End();
        Timer_A1_ms_elapsed = 1U;
        Detect_Line_Position(data);
    }
}

// Maintain elapsed milliseconds.
void SysTick_Handler(void)
{
    system_ms++;
}

// For steering along an ordinary line segment using weighted position. Junction/dead-end recognition must run before calling this helper.
static void Follow_Line(uint8_t data)
{
    int32_t position;

    if (data == 0U)
    {
        Motor_Stop();
        return;
    }

    position = Reflectance_Sensor_Position(data);
    if (position >= 332)
    {
        Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
    }
    else if (position <= -332)
    {
        Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
    }
    else if (position >= 47)
    {
        Motor_Forward(PWM_NOMINAL, PWM_SLOW);
    }
    else if (position <= -47)
    {
        Motor_Forward(PWM_SLOW, PWM_NOMINAL);
    }
    else
    {
        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
    }
}

int main(void)
{
    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the clock
    Clock_Init48MHz();

    // Initialize the built-in red LED and the RGB LED on the microcontroller
    LED1_Init();
    LED2_Init();

    // Initialize the EUSCI_A0_UART module
    EUSCI_A0_UART_Init_Printf();

    // Initialize the 8-Channel QTRX Reflectance Sensor Array module
    Reflectance_Sensor_Init();

    // Initialize the SysTick timer
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Initialize the Timer A1 periodic interrupts every 1 ms
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Initializing the motors
    Motor_Init();
    Motor_Stop();

    // Enable the interrupts used by the modules
    EnableInterrupts();

    while(1)
    {

    }
}
