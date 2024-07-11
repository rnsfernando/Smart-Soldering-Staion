/**
 * -------------------------------------------------------------------------------------+
 * @desc        LCD FONT 5x8
 * -------------------------------------------------------------------------------------+
 * @source      
 *              Copyright (C) 2020 Marian Hrinko.
 *              Written by Marian Hrinko (mato.hrinko@gmail.com)
 *
 * @author      Marian Hrinko
 * @date        08.12.2020
 * @update      08.12.2022
 * @file        font.h
 * @version     1.0
 * @tested      AVR Atmega328p
 *
 * @depend      
 * -------------------------------------------------------------------------------------+
 * @descr       LCD pixel fonts
 * -------------------------------------------------------------------------------------+
 * @usage       Display characters
 * -------------------------------------------------------------------------------------+
 * 
 * Modified by Rebecca Fernando
 * Date: 2024/07/4
 * Description: Edited for specific project requirements
 */

/**
 * -------------------------------------------------------------------------------------+
 * @brief       SSD1306 OLED Driver
 * -------------------------------------------------------------------------------------+
 *              Copyright (C) 2020 Marian Hrinko.
 *              Written by Marian Hrinko (mato.hrinko@gmail.com)
 *
 * @author      Marian Hrinko
 * @date        06.10.2020
 * @file        ssd1306.h
 * @version     2.0.0
 * @test        AVR Atmega328p
 *
 * @depend      string.h, font.h, twi.h
 * -------------------------------------------------------------------------------------+
 * @brief       Version 1.0 -> applicable for 1 display
 *              Version 2.0 -> rebuild to 'cacheMemLcd' array
 *              Version 3.0 -> simplified alphanumeric version for 1 display
 * -------------------------------------------------------------------------------------+
 * @usage       Basic Setup for OLED Display
 * -------------------------------------------------------------------------------------+
 * 
 * Modified by Rebecca Fernando
 * Date: 2024/07/4

/**
 * -------------------------------------------------------------------------------------+
 * @brief       I2C Library
 * -------------------------------------------------------------------------------------+
 *              Copyright (C) 2017 Michael Köhler
 *
 * @author      Michael Köhler
 * @date        09.10.2017
 * @file        i2c.c
 * -------------------------------------------------------------------------------------+
 * 
 * Modified by Rebecca Fernando
 * Date: 2024/07/4
 
/****************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 ****************************************************************
 * 
 * For an ultra-detailed explanation of why the code is the way it is, please visit: 
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 * 
 * For function documentation see: 
 * http://playground.arduino.cc/Code/PIDLibrary
 * (Click "Libraries" on the left panel. The link to the documentation is listed as "PIDLibrary - Provides basic feedback control".)
 ****************************************************************
 * 
 * Modified by Rebecca Fernando
 * Date: 2024/07/4
 *Description: Edited for specific project Solering Station
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/atomic.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

// Pin definitions
#define PB_BACK    PB0   // Pin 8 on Arduino, mapped to Port B, Pin 0 (PB0)
#define PB_OK      PD7   // Pin 7 on Arduino, mapped to Port D, Pin 7 (PD7)
#define PB_UP      PD4   // Pin 4 on Arduino, mapped to Port D, Pin 4 (PD4)
#define PB_DOWN    PD2   // Pin 2 on Arduino, mapped to Port D, Pin 2 (PD2)

#define CONTROL_PIN_IRON    PB2   // Pin 16 on Arduino, mapped to Port B, Pin 2 (PB2)
#define CONTROL_PIN_GUN     PB3   // Pin 17 on Arduino, mapped to Port B, Pin 3 (PB3)

#define TEMP_PIN_IRON       PC1   // Pin A1 on Arduino, mapped to Port C, Pin 1 (PC1)
#define TEMP_PIN_GUN        PC2   // Pin A2 on Arduino, mapped to Port C, Pin 2 (PC2)

// Global variables
bool in_temp_display = false;
bool in_menu1 = false;
bool in_choice = false;

const char* gun_iron[2] = {"Both", "Hot air gun"};
int selected = 0;
const char* menu1[2] = {"Sleep Mode", "Tip Change"};
int select_m1 = 0;

int Setpoint_iron = 150;
int Setpoint_gun = 150;

bool sleeping = false;
volatile unsigned long last_activity_time = 0;
unsigned long sleep_duration = 60000; // 1 minute for testing

double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;
double outMin, outMax;

double Input_iron;
double Output_iron;
double Setpoint;
double Input_gun;
double Output_gun;

void I2C_Init();
void I2C_Start();
void I2C_Write(uint8_t data);
void SCREEN_Command(uint8_t command);
void SCREEN_Data(uint8_t data);
void SCREEN_Init();
void SCREEN_Clear();
void SCREEN_SetCursor(uint8_t row, uint8_t col);
void SCREEN_WriteChar(uint8_t ch, uint8_t size);
void SCREEN_Printf(const char *str, uint8_t size);
void go_to_menu();
void display_temp(int setpoint);
void go_to_menu1();
void run_mode(int mode);
double readTemperature(int pin);
void checkSleepMode();
void wakeUp();
void sleepMode();
void tipChange();
void display_A_Menu(int arrowIndex, const char* Arr[], int Arr_len, int textSize);
int wait_for_button_press();
void millis_init();
unsigned long millis();


void millis_init() {
	// Configure Timer0 for 1ms interrupt using prescaler 64
	TCCR0A = 0x00;             // Normal mode
	TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
	TIMSK0 = (1 << TOIE0);     // Enable overflow interrupt for Timer0
	TCNT0 = 0;                 // Initialize counter
}

// Interrupt Service Routine for Timer0 Overflow
ISR(TIMER0_OVF_vect) {
	last_activity_time++; // Increment millis_count in the ISR
}

unsigned long millis()
{
	unsigned long millis_value;
	// Ensure atomic access to millis_count
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		millis_value = last_activity_time;
	}
	return millis_value;
}

// Font array for ASCII characters
const uint8_t font[][8] = {
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Space (32)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // ! (33)
	{0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00},  // ' (39)
	{0x00, 0x1C, 0x1C, 0x00, 0x1C, 0x1C, 0x00, 0x00},  // ( (40)
	{0x00, 0x1C, 0x1C, 0x00, 0x1C, 0x1C, 0x00, 0x00},  // ) (41)
	{0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00},  // , (44)
	{0x00, 0x00, 0x00, 0x1C, 0x1C, 0x00, 0x00, 0x00},  // - (45)
	{0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00},  // . (46)
	{0x00, 0x00, 0x00, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // / (47)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // 0 (48)
	{0x00, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x00},  // 1 (49)
	{0x00, 0x1C, 0x07, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // 2 (50)
	{0x00, 0x1C, 0x07, 0x00, 0x07, 0x1C, 0x00, 0x00},  // 3 (51)
	{0x00, 0x07, 0x07, 0x1C, 0x07, 0x07, 0x00, 0x00},  // 4 (52)
	{0x00, 0x1C, 0x1C, 0x00, 0x1C, 0x07, 0x00, 0x00},  // 5 (53)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // 6 (54)
	{0x00, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x00},  // 7 (55)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // 8 (56)
	{0x00, 0x1C, 0x1C, 0x00, 0x1C, 0x1C, 0x00, 0x00},  // 9 (57)
	{0x00, 0x00, 0x1C, 0x1C, 0x1C, 0x00, 0x00, 0x00},  // : (58)
	{0x00, 0x00, 0x00, 0x1C, 0x1C, 0x00, 0x00, 0x00},  // ; (59)
	{0x00, 0x00, 0x07, 0x1C, 0x07, 0x00, 0x00, 0x00},  // < (60)
	{0x00, 0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // = (61)
	{0x00, 0x00, 0x07, 0x1C, 0x07, 0x00, 0x00, 0x00},  // > (62)
	{0x00, 0x00, 0x1C, 0x07, 0x1C, 0x00, 0x00, 0x00},  //? (63)
	{0x00, 0x07, 0x07, 0x1C, 0x07, 0x07, 0x00, 0x00},  // A (65)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // B (66)
	{0x00, 0x1C, 0x07, 0x07, 0x07, 0x1C, 0x00, 0x00},  // C (67)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // D (68)
	{0x00, 0x1C, 0x1C, 0x00, 0x1C, 0x1C, 0x00, 0x00},  // E (69)
	{0x00, 0x1C, 0x1C, 0x00, 0x1C, 0x1C, 0x00, 0x00},  // F (70)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // G (71)
	{0x00, 0x07, 0x07, 0x1C, 0x07, 0x07, 0x00, 0x00},  // H (72)
	{0x00, 0x1C, 0x00, 0x1C, 0x00, 0x1C, 0x00, 0x00},  // I (73)
	{0x00, 0x1C, 0x07, 0x07, 0x07, 0x07, 0x00, 0x00},  // J (74)
	{0x00, 0x07, 0x1C, 0x1C, 0x1C, 0x07, 0x00, 0x00},  // K (75)
	{0x00, 0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // L (76)
	{0x00, 0x07, 0x1C, 0x1C, 0x07, 0x07, 0x00, 0x00},  // M (77)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // N (78)
	{0x00, 0x1C, 0x07, 0x07, 0x07, 0x1C, 0x00, 0x00},  // O (79)
	{0x00, 0x07, 0x07, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // P (80)
	{0x00, 0x1C, 0x07, 0x07, 0x07, 0x07, 0x00, 0x00},  // Q (81)
	{0x00, 0x1C, 0x1C, 0x07, 0x1C, 0x07, 0x00, 0x00},  // R (82)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // S (83)
	{0x00, 0x1C, 0x00, 0x1C, 0x00, 0x1C, 0x00, 0x00},  // T (84)
	{0x00, 0x1C, 0x07, 0x07, 0x07, 0x1C, 0x00, 0x00},  // U (85)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // V (86)
	{0x00, 0x1C, 0x1C, 0x07, 0x1C, 0x07, 0x00, 0x00},  // W (87)
	{0x00, 0x07, 0x1C, 0x1C, 0x1C, 0x07, 0x00, 0x00},  // X (88)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // Y (89)
	{0x00, 0x1C, 0x07, 0x1C, 0x07, 0x1C, 0x00, 0x00},  // Z (90)
	{0x00, 0x1C, 0x07, 0x1C, 0x07, 0x1C, 0x00, 0x00},  // a (97)
	{0x00, 0x07, 0x1C, 0x1C, 0x1C, 0x07, 0x00, 0x00},  // b (98)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // c (99)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // d (100)
	{0x00, 0x1C, 0x1C, 0x00, 0x1C, 0x1C, 0x00, 0x00},  // e (101)
	{0x00, 0x1C, 0x1C, 0x00, 0x1C, 0x1C, 0x00, 0x00},  // f (102)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // g (103)
	{0x00, 0x07, 0x07, 0x1C, 0x07, 0x07, 0x00, 0x00},  // h (104)
	{0x00, 0x1C, 0x00, 0x1C, 0x00, 0x1C, 0x00, 0x00},  // i (105)
	{0x00, 0x1C, 0x07, 0x07, 0x07, 0x07, 0x00, 0x00},  // j (106)
	{0x00, 0x07, 0x1C, 0x1C, 0x1C, 0x07, 0x00, 0x00},  // k (107)
	{0x00, 0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // l (108)
	{0x00, 0x07, 0x1C, 0x1C, 0x07, 0x07, 0x00, 0x00},  // m (109)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // n (110)
	{0x00, 0x1C, 0x07, 0x07, 0x07, 0x1C, 0x00, 0x00},  // o (111)
	{0x00, 0x07, 0x07, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // p (112)
	{0x00, 0x1C, 0x07, 0x07, 0x07, 0x07, 0x00, 0x00},  // q (113)
	{0x00, 0x1C, 0x1C, 0x07, 0x1C, 0x07, 0x00, 0x00},  // r (114)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // s (115)
	{0x00, 0x1C, 0x00, 0x1C, 0x00, 0x1C, 0x00, 0x00},  // t (116)
	{0x00, 0x1C, 0x07, 0x07, 0x07, 0x1C, 0x00, 0x00},  // u (117)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // v (118)
	{0x00, 0x1C, 0x1C, 0x07, 0x1C, 0x07, 0x00, 0x00},  // w (119)
	{0x00, 0x07, 0x1C, 0x1C, 0x1C, 0x07, 0x00, 0x00},  // x (120)
	{0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00},  // y (121)
	{0x00, 0x1C, 0x07, 0x1C, 0x07, 0x1C, 0x00, 0x00},  // z (122)
};

void I2C_Init() {
	// Initialize I2C with 100 kHz baud rate
	TWBR = 72;  // Calculated value for 100 kHz at 16 MHz CPU clock
	TWSR &= ~(1<<TWPS0) & ~(1<<TWPS1);  // Prescaler = 1
}

void I2C_Start() {
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);  // Start condition
	while (!(TWCR & (1<<TWINT)));  // Wait for start condition to be transmitted
}

void I2C_Write(uint8_t data) {
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);  // Clear TWINT to start transmission of data
	while (!(TWCR & (1<<TWINT)));  // Wait for transmission to complete
}

void SCREEN_Command(uint8_t command) {
	I2C_Start();
	I2C_Write(SCREEN_ADDRESS << 1);  // Slave address + write mode
	I2C_Write(0x00);            // Command mode
	I2C_Write(command);
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);  // Stop condition
	_delay_us(10);  // Small delay
}

void SCREEN_Data(uint8_t data) {
	I2C_Start();
	I2C_Write(SCREEN_ADDRESS << 1);  // Slave address + write mode
	I2C_Write(0x40);            // Data mode
	I2C_Write(data);
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);  // Stop condition
	_delay_us(10);  // Small delay
}

void SCREEN_Init() {
	I2C_Init();  // Initialize I2C communication
	_delay_ms(100);  // Delay for stable power-on
	SCREEN_Command(0xAE);  // Display off
	SCREEN_Command(0x20);  // Set Memory Addressing Mode
	SCREEN_Command(0x10);  // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;
	// 10,Page Addressing Mode (RESET);11,Invalid
	SCREEN_Command(0xB0);  // Set Page Start Address for Page Addressing Mode,0-7
	SCREEN_Command(0xC8);  // Set COM Output Scan Direction
	SCREEN_Command(0x00);  // ---set low column address
	SCREEN_Command(0x10);  // ---set high column address
	SCREEN_Command(0x40);  // --set start line address
	SCREEN_Command(0x81);  // Set contrast control register
	SCREEN_Command(0xFF);  // --set segment re-map 0 to 127
	SCREEN_Command(0xA1);  // Set Display RAM Horizontal Flip (0xA0,0xA1)
	SCREEN_Command(0xA6);  // Set display mode (0xA6,0xA7)
	SCREEN_Command(0xA8);  // Set Multiplex Ratio (1 to 64)
	SCREEN_Command(0x3F);  // 1/64 duty
	SCREEN_Command(0xA4);  // Output follows RAM content;0xa4;Output ignores RAM content
	SCREEN_Command(0xD3);  //-set display offset
	SCREEN_Command(0x00);   //-not offset
	SCREEN_Command(0xD5);  //--set display clock divide ratio/oscillator frequency
	SCREEN_Command(0xF0);  //--set divide ratio, Set Clock as 100 Frames/Sec
	SCREEN_Command(0xD9);  //--set pre-charge period
	SCREEN_Command(0x22);   //
	SCREEN_Command(0xDA);  //--set com pins hardware configuration
	SCREEN_Command(0x12);
	SCREEN_Command(0xDB);  //--set vcomh
	SCREEN_Command(0x20);  //0x20,0.77xVcc
	SCREEN_Command(0x8D);  //--set DC-DC enable
	SCREEN_Command(0x14);
	SCREEN_Command(0xAF);  //--turn on SCREEN panel
	SCREEN_Command(0xAF);
	SCREEN_Command(0xB1);  // Set Phase Length
	SCREEN_Command(0x1D);  // Select External Vcc Supply

	SCREEN_Command(0xB3);  // Set Display Clock Divide Ratio/Oscillator Frequency
	SCREEN_Command(0xF1);  // Set Display Divide Ratio 3e, 3f
}

void SCREEN_Clear() {
	uint8_t i, j;
	for (j = 0; j < 8; j++) {
		SCREEN_Command(0xB0 + j);  // Set page address (0 to 7)
		SCREEN_Command(0x00);      // Set low column address
		SCREEN_Command(0x10);      // Set high column address
		for (i = 0; i < 128; i++) {
			SCREEN_Data(0x00);     // Clear entire page
		}
	}
}

void SCREEN_SetCursor(uint8_t row, uint8_t col) {
	SCREEN_Command(0xB0 + row);            // Set page address
	SCREEN_Command(0x00 + (8*col & 0x0F)); // Set column lower address
	SCREEN_Command(0x10 + ((8*col>>4)&0x0F));// Set column higher address
}

void SCREEN_WriteChar(uint8_t ch, uint8_t size) {
	uint8_t i, j;
	for (i = 0; i < 8; i++) {
		for (j = 0; j < size; j++) {
			SCREEN_Data(font[ch - 32][i]);  // Print each row of the character
		}
	}
}

void SCREEN_Printf(const char *str, uint8_t size) {
	while (*str) {
		SCREEN_WriteChar(*str++, size);
	}
}

// PID control class 
typedef struct {
	double *myInput;    // pointer to input value
	double *myOutput;   // pointer to output value
	double *mySetpoint; // pointer to setpoint value
	
	double kp;          // proportional gain
	double ki;          // integral gain
	double kd;          // derivative gain
	
	bool inAuto;        // flag indicating whether the controller is in automatic mode
	unsigned long SampleTime; // sample time in milliseconds
	unsigned long lastTime;   // last time the controller was updated
	
	double outputSum;   // sum of errors for integral term
	double lastInput;   // last input for derivative term
	
	double outMin;      // minimum output limit
	double outMax;      // maximum output limit
} PID;

void PID_Init(PID *pid, double *input, double *output, double *setpoint, double Kp, double Ki, double Kd, int direction) {
	pid->myInput = input;
	pid->myOutput = output;
	pid->mySetpoint = setpoint;
	pid->kp = Kp;
	pid->ki = Ki;
	pid->kd = Kd;
	pid->inAuto = true;
	pid->SampleTime = 100;
	pid->lastTime = millis() - pid->SampleTime;
	pid->outputSum = 0;
	pid->lastInput = 0;
}

void PID_Compute(PID *pid) {
	if (!pid->inAuto) return;
	
	unsigned long now = millis();
	unsigned long timeChange = (now - pid->lastTime);
	if (timeChange >= pid->SampleTime) {
		double input = *(pid->myInput);
		double error = *(pid->mySetpoint) - input;
		pid->outputSum += (pid->ki * error);
		
		double output = pid->kp * error + pid->outputSum - pid->kd * (input - pid->lastInput);
		*(pid->myOutput) = output;
		
		if (*(pid->myOutput) > pid->outMax) *(pid->myOutput) = pid->outMax;
		else if (*(pid->myOutput) < pid->outMin) *(pid->myOutput) = pid->outMin;
		
		pid->lastInput = input;
		pid->lastTime = now;
	}
}

void PID_SetTunings(PID *pid,double Kp,double Ki,double Kd) {
	pid->kp = Kp;
	pid->ki = Ki;
	pid->kd = Kd;
}

void PID_SetOutputLimits(PID *pid,double Min,double Max) {
	if (Min >= Max) return;
	pid->outMin = Min;
	pid->outMax = Max;
	
	if (*(pid->myOutput) > pid->outMax) *(pid->myOutput) = pid->outMax;
	else if (*(pid->myOutput) < pid->outMin) *(pid->myOutput) = pid->outMin;
	
	if (pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
	else if (pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;
}

void PID_SetMode(PID *pid, int Mode) {
	pid->inAuto = (Mode == 1);
}

PID myPIDIron, myPIDGun;

void setup()
{
	// Initialize GPIO pins
	DDRB &= ~(1 << PB_BACK);    // PB_BACK_PIN as input
	DDRD &= ~((1 << PB_OK) | (1 << PB_UP) | (1 << PB_DOWN));  // PB_OK_PIN, PB_UP_PIN, PB_DOWN_PIN as inputs

	DDRB |= (1 << CONTROL_PIN_IRON) | (1 << CONTROL_PIN_GUN);  // CONTROL_PIN_IRON, CONTROL_PIN_GUN as outputs

	// Enable pull-up resistors so that the pin reads as high when the button is not pressed.
	PORTB |= (1 << PB_BACK);    // PB_BACK_PIN with pull-up
	PORTD |= (1 << PB_OK) | (1 << PB_UP) | (1 << PB_DOWN);  // PB_OK_PIN, PB_UP_PIN, PB_DOWN_PIN with pull-ups
	
	DDRC &= ~((1 << TEMP_PIN_IRON) | (1 << TEMP_PIN_GUN));  // Set TEMP_PIN_IRON and TEMP_PIN_GUN as inputs
	PORTC |= (1 << TEMP_PIN_IRON) | (1 << TEMP_PIN_GUN);   // Enable pull-up resistors for these pins
	
	// Initialize ADC for analogRead
	ADMUX |= (1 << REFS0); // Set reference voltage to AVcc
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128
	ADCSRA |= (1 << ADEN); // Enable ADC

	// Initialize PWM for analogWrite
	// Timer 1 for CONTROL_PIN_IRON (PB2)
	TCCR1A |= (1 << WGM10) | (1 << WGM11) | (1 << COM1B1); // Fast PWM, 8-bit
	TCCR1B |= (1 << WGM12) | (1 << CS11); // Fast PWM, prescaler 8

	// Timer 2 for CONTROL_PIN_GUN (PB3)
	TCCR2A |= (1 << WGM20) | (1 << WGM21) | (1 << COM2A1); // Fast PWM
	TCCR2B |= (1 << CS22); // Prescaler 64
	
	// Initialize necessary components
	millis_init();
	sei(); // Enable global interrupts
	
	// Set up PID controller
	PID_Init(&myPIDIron,&Input_iron, &Output_iron, &Setpoint,consKp,consKi,consKd,1);
	PID_Init(&myPIDGun,&Input_gun, &Output_gun, &Setpoint,consKp,consKi,consKd,1);
	PID_SetOutputLimits(&myPIDIron, 0,225);
	PID_SetMode(&myPIDIron, 1);// AUTOMATIC mode
	PID_SetOutputLimits(&myPIDGun, 0,225);
	PID_SetMode(&myPIDGun, 1);// AUTOMATIC mode
	
	last_activity_time = millis();
	
	eeprom_update_word((uint16_t*)0, Setpoint_iron);
	
	SCREEN_Init();    // Initialize the SCREEN
	SCREEN_Clear();   // Clear the display
	// Display "Welcome!" message
	SCREEN_SetCursor(0, 0);      // Set cursor to start of first line
	SCREEN_Printf("Welcome!",3);         // Print text
	_delay_ms(3000);
	SCREEN_Clear();
	
	in_choice = true;
}

void loop()
{
	if (in_choice) {
		go_to_menu();
	}
	if (selected && in_temp_display) {
		display_temp(Setpoint_gun);
		} else if (!selected && in_temp_display) {
		int set = eeprom_read_word((uint16_t*)0); // Read Setpoint_iron from EEPROM
		display_temp(set);
	}

	if (in_menu1) {
		go_to_menu1();
	}
	
	checkSleepMode(); // Check for sleep mode based on inactivity
	
	// PID for iron
	Input_iron = readTemperature(TEMP_PIN_IRON);
	int fSetpointIron = eeprom_read_word((uint16_t*)0);
	double gapIron = abs(fSetpointIron - Input_iron);

	if (gapIron < 10) {
		PID_SetTunings(&myPIDIron,consKp,consKi,consKd);
		} else {
		PID_SetTunings(&myPIDIron,aggKp,aggKi,aggKd);
	}

	PID_Compute(&myPIDIron);
	OCR1B =Output_iron;//set PWM duty cycle on PB2 (analogWrite(controlPinIron, Output_iron);)

	// PID for hot air gun
	Input_gun = readTemperature(TEMP_PIN_GUN);
	double gapGun = abs(Setpoint_gun - Input_gun);

	if (gapGun < 10) {
		PID_SetTunings(&myPIDGun, consKp, consKi, consKd);
		} else {
		PID_SetTunings(&myPIDGun, aggKp, aggKi, aggKd);
	}

	PID_Compute(&myPIDGun);
	OCR2A =Output_gun;//set PWM duty cycle on PB3 (analogWrite(controlPinGun, Output_gun);)
}

int main()
{
	setup();
	while (1)
	{
		loop();
	}
}

void display_A_Menu(int arrowIndex, const char* Arr[], int Arr_len, int textSize) {
	SCREEN_Clear();
	for (int i = 0; i < Arr_len; i++) {
		SCREEN_SetCursor(i, 0);
		if (i == arrowIndex) {
			SCREEN_Printf("> ",1);
			} else {
			SCREEN_Printf("  ",1);
		}
		SCREEN_Printf(Arr[i],1);
	}
}

int wait_for_button_press()
{
	while (1) {
		if (!(PIND & (1 << PB_UP))) {
			_delay_ms(200);
			return PB_UP;
		}
		if (!(PIND & (1 << PB_DOWN))) {
			_delay_ms(200);
			return PB_DOWN;
		}
		if (!(PIND & (1 << PB_OK))) {
			_delay_ms(200);
			return PB_OK;
		}
		if (!(PIND & (1 << PB_BACK))) {
			_delay_ms(200);
			return PB_BACK;
		}
	}
}

void go_to_menu()
{
	while (PIND & (1 << PB_BACK)) {
		SCREEN_Clear();
		display_A_Menu(selected, gun_iron, 2, 1);
		int pressed = wait_for_button_press();
		if (pressed == PB_UP) {
			_delay_ms(200);
			selected = (selected + 1) % 2;
			display_A_Menu(selected, gun_iron, 2, 1);
			} else if (pressed == PB_DOWN) {
			_delay_ms(200);
			selected = (selected - 1 + 2) % 2;
			display_A_Menu(selected, gun_iron, 2, 1);
			} else if (pressed == PB_OK) {
			_delay_ms(200);
			in_choice = false;
			in_temp_display = true;
			last_activity_time = millis(); // Update activity time
			break;
			} else if (pressed == PB_BACK) {
			_delay_ms(200);
			// Do something if needed
		}
	}
}

void display_temp(int setpoint) {
	SCREEN_Clear();
	SCREEN_SetCursor(0,20);

	if (selected) {
		SCREEN_Printf( "Set Gun Temp: ",2);
		} else {
		SCREEN_Printf(  "Set Iron Temp: ",2);
	}

	char* temp_str = malloc(12);
	snprintf(temp_str, 12, "%d", setpoint);
	const char* const_temp_str = temp_str;

	SCREEN_Printf(const_temp_str,2);
	SCREEN_WriteChar('c',2);


	int pressed = wait_for_button_press();
	if (pressed == PB_BACK) {
		_delay_ms(200);
		in_choice = true;
		} else if (pressed == PB_UP) {
		_delay_ms(200);
		if (selected) {
			Setpoint_gun += 1;
			if (Setpoint_gun > 400) {
				Setpoint_gun = 400;
			}
			} else {
			setpoint += 1;
			if (setpoint > 400) {
				setpoint = 400;
			}
			eeprom_update_word((uint16_t*)0, setpoint);
		}
		last_activity_time = millis(); // Update activity time
		} else if (pressed == PB_DOWN) {
		_delay_ms(200);
		if (selected) {
			Setpoint_gun -= 1;
			if (Setpoint_gun < 150) {
				Setpoint_gun = 150;
			}
			} else {
			setpoint -= 1;
			if (setpoint < 150) {
				setpoint = 150;
			}
			eeprom_update_word((uint16_t*)0, setpoint);
		}
		last_activity_time = millis(); // Update activity time
		} else if (pressed == PB_OK) {
		_delay_ms(200);
		if (!selected) {
			in_choice = false;
			in_temp_display = false;
			in_menu1 = true;
		}
	}
}

void go_to_menu1() {
	while (PIND & (1 << PB_BACK)) {
		SCREEN_Clear();
		display_A_Menu(select_m1, menu1, 2, 1);
		int pressed = wait_for_button_press();
		if (pressed == PB_UP) {
			_delay_ms(200);
			select_m1 = (select_m1 + 1) % 2;
			display_A_Menu(select_m1, menu1, 2, 1);
			} else if (pressed == PB_DOWN) {
			_delay_ms(200);
			select_m1 = (select_m1 - 1 + 2) % 2;
			display_A_Menu(select_m1, menu1, 2, 1);
			} else if (pressed == PB_OK) {
			_delay_ms(200);
			int mode = select_m1;
			run_mode(mode);
			} else if (pressed == PB_BACK) {
			_delay_ms(200);
			in_choice = false;
			in_menu1 = false;
			in_temp_display = true;
			break;
		}
	}
}

void run_mode(int mode) {
	if (mode == 0) {
		sleepMode();
		} else if (mode == 1) {
		tipChange();
	}
}

void tipChange() {
	SCREEN_Clear();
	SCREEN_SetCursor(0,20);
	SCREEN_Printf("Tip Changed",2);
	_delay_ms(1000);
	// Lower temperature for safe tip change
	int original_setpoint = eeprom_read_word((uint16_t*)0); // Read from EEPROM
	Setpoint_iron = 100; // Example lower temperature
	eeprom_update_word((uint16_t*)0, Setpoint_iron);
	_delay_ms(5000); // Wait for 5 seconds
	Setpoint_iron = original_setpoint;
	eeprom_update_word((uint16_t*)0, Setpoint_iron);
	SCREEN_Clear();
}

void checkSleepMode() {
	if (millis() - last_activity_time > sleep_duration && !sleeping) {
		sleepMode();
		sleeping = true;
	}
}

void wakeUp() {
	SCREEN_Clear();
	SCREEN_SetCursor(0,20);
	SCREEN_Printf("Waking up...",2);
	_delay_ms(1000);
	sleeping = false;
	last_activity_time = millis();
	SCREEN_Clear();
}

void sleepMode() {
	SCREEN_Clear();
	SCREEN_SetCursor(0,20);
	SCREEN_Printf("Sleeping...",2);
	_delay_ms(1000);
	// Enter low power mode or stop heating elements
	OCR0A = 0;  // Stop heating elements of iron
	OCR0B = 0;  // Stop heating elements of gun
	while (1) {
		if (!(PIND & (1 << PB_OK)) || !(PIND & (1 << PB_UP)) || !(PIND & (1 << PB_DOWN)) || !(PIND & (1 << PB_BACK))) {
			wakeUp();
			break;
		}
	}
}

double readTemperature(int pin) {
	// Select ADC channel
	ADMUX = (ADMUX & 0xF0) | (pin & 0x0F);
	// Start conversion
	ADCSRA |= (1 << ADSC);
	// Wait for conversion to complete
	while (ADCSRA & (1 << ADSC));
	double input = ADC;
	input = ((input / 1024.0) * 500.0);  // Assuming a 5V reference
	
	// Map the input value from 0-450 to 25-350
	double mappedInput = (input - 0) * (350.0 - 25.0) / (450.0 - 0) + 25.0;
	return mappedInput;
}



