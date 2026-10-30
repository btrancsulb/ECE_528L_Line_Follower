# ECE 528/L - Robotics and Embedded Systems with Lab
**CSU Northridge**

**Department of Electrical and Computer Engineering**

## Line Follower Lab

### Overview

The Line Follower Lab introduces a line-following algorithm, combining  reflectance sensing, periodic interrupts, and motor control to the MSP432 LaunchPad and a TI-RSLK MAX Chassis. The code uses a QTRX sensor array to detect black tape on a white surface. Once the tape is notices, the code then uses those readings to drive the robot forward, left, or right. The robot corrects itself and stops given the currect line pattern.

By the completion of this lab, we were able to:
* Build upon the previous Reflectance Sensor and Motor Control labs.
* Use Timer A1 to schedult sensor measurements and steering updates
* Use Timer A0 to generate the motor PWM signals and use SysTick to maintain a milisecond counter
* Create an algorithm that follows a basic line that stops at intersections, or dead ends

### PreLab Assignments
```
void Chassis_Board_LEDs_Init(void){
    P8->DIR  |= 0xE1;
    P8->OUT  &= ~(0xE1);
    P8->SEL0 & = ~(0xE1);
    P8->SEL1 & = ~(0xE1);
}
```
```
void Print_Tenth(void)
{
    int index;
    for (index = 10; index <= 51; index++)
    {
        if ((index % 10) == 0)
        {
            printf("Index %d: Every 10th iteration.\n", index);
        }
        else if ((index % 10) == 1)
        {
            printf("Index %d: Following the 10th iteration.\n", index);
        }
    }
}

```
### Components Used

| Component | Quantity | Manufacturer |
|---|---|---|
| MSP432P401R LaunchPad | 1 | Texas Instruments |
| TI-RSLK MAX chassis with motor-driver board | 1 | Pololu |
| Pololu gearmotors with encoders | 2 | Pololu |
| Pololu 8-Channel QTRX Sensor Array | 1 | Pololu |
| LaunchPad LED1 and RGB LED | Built in | Texas Instruments |
| USB-A to Micro-USB cable | 1 | N/A |
| AA batteries | 6 | N/A |
| Black tape | N/A | N/A  |
| Computer with Code Composer Studio | N/A | N/A |

**Main signal connections**

| Signal | LaunchPad pin(s) |
|---|---|
| Reflectance sensor outputs 0–7 | P7.0–P7.7 |
| Even / odd IR emitter controls | P5.3 / P9.2 |
| Sensor supply / ground through the chassis connection | 5 V / GND |
| Right / left motor PWM | P2.6 / P2.7 |
| Left / right motor direction | P5.4 / P5.5 |
| Motor enable controls | P3.6–P3.7 |
| Red LED1 | P1.0 |
| RGB LED | P2.0–P2.2 |

### Analysis and Results

The results below describe the current source code and calculations from its configured values. 

### Video

**Sensor sampling and control timing**

`Timer_A1_Periodic_Task()` runs nominally every 1 ms. At tick 10, `Reflectance_Sensor_Start()` enables the IR emitters, drives all eight sensor outputs high for 10 µs, and changes them back to inputs. At tick 11, `Reflectance_Sensor_End()` reads Port 7 and disables the emitters. A high bit represents black tape; a low bit represents white background.

After reading, the counter resets to 1, so subsequent starts occur at approximately 20, 30, and 40 ms, with reads at 21, 31, and 41 ms. The resulting sensor and steering update rate is approximately **100 Hz**, even though the interrupt itself runs at 1 kHz. The discharge interval is approximately 1 ms, slightly shorter because charging happens inside the preceding interrupt.

```text
Time (in ms):       10      11              20       21
Sensor action:   Start -> Read            Start -> Read
Control action:           LEDs + motors            LEDs + motors
                          <---------- 10 ms! --------->
```

**Position calculation and steering**

`Reflectance_Sensor_Position()` averages the weights of the active sensor bits:

```text
Bit:       7     6     5    4    3    2    1    0
Weight: -334  -238  -142  -48   48  142  238  334

Position = sum of active weights / number of active sensors
```

| Calculated position | Command |
|---|---|
| At least +332 | Pivot right |
| +47 through +331 | Forward with the right wheel slower |
| -46 through +46 | Forward with equal motor commands |
| -331 through -47 | Forward with the left wheel slower |
| At most -332 | Pivot left |

**LED and motor results from the current implementation**

Left and right in this table describe the tape position relative to the sensor array. Motor arguments are listed as `(left, right)` and are PWM compare values, not percentages.

| Pattern | Sensor mask | Position | LED1 | RGB LED | Motor command |
|---|---|---:|---|---|---|
| Center | `0001_1000` (`0x18`) | 0 | Off | Green | Forward `(2500, 2500)` |
| Tape slightly left | `0011_1000` (`0x38`) | -47 | Off | Yellow | Forward `(1500, 2500)` |
| Tape slightly right | `0001_1100` (`0x1C`) | +47 | Off | Pink | Forward `(2500, 1500)` |
| Tape far left | `1000_0000` (`0x80`) | -334 | Off | White | Pivot left `(2500, 2500)` |
| Tape far right | `0000_0001` (`0x01`) | +334 | Off | Sky blue | Pivot right `(2500, 2500)` |
| Left turn | `1111_1000` (`0xF8`) | - | On | White | Pivot left `(2500, 2500)` |
| Right turn | `0001_1111` (`0x1F`) | - | On | Sky blue | Pivot right `(2500, 2500)` |
| T-intersection | `1111_1111` (`0xFF`) | - | Off | Blue | Stop |
| Dead end / no tape | `0000_0000` (`0x00`) | - | Off | Red | Stop |
| Any other mask | Other nonzero values | Calculated | Off | Off | Steer using the position thresholds |

Unlike the earlier sensor-only lab, an unrecognized nonzero pattern still produces a steering command. For example, `0x08` has position +48 and causes a gentle right correction even though the RGB LED is off.

**Motor timing calculations**

Timer A0 uses the configured 12 MHz SMCLK divided by 8, a period constant of 15000, and up/down counting:

```text
Timer clock = 12 MHz / 8 = 1.5 MHz
PWM period  = (2 × 15000) / 1,500,000 = 20 ms
PWM rate    = 1 / 0.020 = 50 Hz
Nominal command ratio = 2500 / 15000 = 16.7%
Slow command ratio    = 1500 / 15000 = 10.0%
```

These are calculated settings from the driver, not measured wheel speeds. `Motor_Left()` reverses the left motor and drives the right motor forward; `Motor_Right()` does the opposite. `Motor_Stop()` disables both motors and clears their PWM compare values.

### Known Issues or Limitations

- **No distinction between a dead end and a lost line.** Both can produce `0x00`, which stops the motors immediately. There is no search or recovery behavior. A stop is not latched: a later valid reading can command movement again.
- **Turn and intersection recognition requires exact masks.** A partially detected junction may produce an ordinary steering correction instead of the intended turn or stop. There is no filtering amongst consecutive samples.

### Author Contribution
                   
| Lab Report Contributions| Group Member | Implementation Contributions | Group Member |
| ---- | ------------ | ------- | ------- |
|Overview | Feranmi | Source Code | Brandon & Feranmi |
| Components Used | Feranmi | Robot Demonstration | Brandon & Feranmi |
| Analysis and Results | Brandon & Feranmi | Known Issues and Limitations | Brandon |
| Author Contribution | Feranmi | 
References | Feranmi |


### References
1. Aaron Nanas, [*ECE 528/L Lab 3: Line Follower*, Fall 2026](ece528L_fall26_lab3_line_follower.pdf). 
2. [Line Follower main program](ece528L_line_follower/Line_Follower/main.c). 
3. [Reflectance sensor driver](ece528L_line_follower/Line_Follower/src/Reflectance_Sensor.c). 
4. [Motor driver](ece528L_line_follower/Line_Follower/src/Motor.c), [Timer A0 PWM driver](ece528L_line_follower/Line_Follower/src/Timer_A0_PWM.c), and [PWM constants](ece528L_line_follower/Line_Follower/inc/Timer_A0_PWM.h).
5. [Timer A1 interrupt driver](ece528L_line_follower/Line_Follower/src/Timer_A1_Interrupt.c) and [interrupt constants](ece528L_line_follower/Line_Follower/inc/Timer_A1_Interrupt.h).
6. [Reflectance Sensor lab report](../ECE_528L_Reflectance_Sensor/README.md).
7. [Motor Control lab report](../ECE_528L_Motor_Control/README.md). 
8. [GPIO lab report](../ece_528L_gpio/README.md).
