//Martin Bildhjerd

#ifndef CONFIG_H_
#define CONFIG_H_

#ifndef F_CPU
#define F_CPU 16000000UL  //16MHz CPU frequency
#endif

#define BAUD_RATE 115200
#define LCD_RS PB0
#define LCD_EN PB1
#define LIDAR PD0
#define PWM_TOP 39999u
#define SERVO_MIN 1999u
#define SERVO_MAX 4999u
#define DISTANCE_THRESHOLD 600
#define LCD_DELAY_ONE 150
#define LCD_DELAY_TWO 10

#endif
