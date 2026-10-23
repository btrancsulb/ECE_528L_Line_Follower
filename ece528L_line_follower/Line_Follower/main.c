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
#include "inc/Systick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"

// Initializing constant PWM duty cycle values for the motors
#define PWM_NOMIAL 2500

// Initialize a global variable for Timer A1 to keep track of elapsed time in milliseconds
static uint32_t Timer_A1_ms_elapsed = 0;

typedef enum{
    CENTER              = 0b00011000,
    SLIGHT_LEFT_CENTER  = 0b00011100,
    SLIGHT_RIGHT_CENTER = 0b00111000,
    FAR_LEFT            = 0b00000001,
    FAR_RIGHT           = 0b10000000,
    LEFT_TURN           = 0b11111000,
    RIGHT_TURN          = 0B00011111,
    T_INTERSECTION      = 0b11111111,
    DEAD_END            = 0b00000000
} Line_Position;

void Detect_Line_Position(uint8_t reflectance_sensor_data)
{
    switch(reflectance_sensor_data){
    case CENTER:
    {
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_GREEN);
    }
    break;

    case SLIGHT_LEFT_CENTER:
    {
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_YELLOW);
    }
    break;
    case SLIGHT_RIGHT_CENTER:
    {
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_PINK);
    }
    break;
    case FAR_LEFT:
    {
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_WHITE);
    }
    break;
    case FAR_RIGHT:
    {
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_SKY_BLUE);
    }
    break;
    case LEFT_TURN:
    {
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_WHITE);
    }
    break;
    case RIGHT_TURN:
    {
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_SKY_BLUE);
    }
    break;
    case T_INTERSECTION:
    {
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_BLUE);
    }
    break;
    case DEAD_END:
    {
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_RED);
    }
    break;

    default:
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_OFF);
        break;
    }
}

void Timer_A1_Periodic_Task(void){
    // Increment Time_A1_ms_elapsed by 1 every time the Timer A1 periodic interrupt occurs
    Timer_A1_ms_elapsed++;

    // Start the process of reading the reflectance sensor array every 10 ms (i.e. 10, 20, 30, ...)
    if ((Timer_A1_ms_elapsed % 10) == 0) {
        Reflectance_Sensor_Start();
    }

    // Finish reading the reflectance sensor array after 1 ms (i.e. 11, 21, 31, ..)
    if ((Timer_A1_ms_elapsed % 10) == 1) {
        uint8_t Reflectance_Sensor_Data = Reflectance_Sensor_End();
        Detect_Line_Position(Reflectance_Sensor_Data);
    }
}
int main(void)
{

    while(1)
    {

    }
}
