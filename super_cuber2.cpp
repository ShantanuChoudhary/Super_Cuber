/*
 * GccApplication3.cpp
 *
 * Created: 2/28/2015 2:28:39 PM
 *  Author: Rishi Vanukuru
 */ 

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <ctype.h>

/********************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010
 
 This experiment demonstrates Servo motor control using 10 bit fast PWM mode.

 Concepts covered:  Use of timer to generate PWM for servo motor control

 Fire Bird V ATMEGA2560 microcontroller board has connection for 3 servo motors (S1, S2, S3).
 Servo motors move between 0 to 180 degrees proportional to the pulse train 
 with the on time of 0.6 to 2 ms with the frequency between 40 to 60 Hz. 50Hz is most recommended.

 We are using Timer 1 at 10 bit fast PWM mode to generate servo control waveform.
 In this mode servo motors can be controlled with the angular resolution of 1.86 degrees.
 Although angular resolution is less this is very simple method.
 
 There are better ways to produce very high resolution PWM but it involves interrupts at the frequency of the PWM.
 High resolution PWM is used for servo control in the Hexapod robot.
  
 Connection Details:	PORTB 5 OC1A --> Servo 1: Camera pod pan servo
 						PORTB 6 OC1B --> Servo 2: Camera pod tile servo
						PORTB 7 OC1C --> Servo 3: Reserved 
 					  	
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)

 2. 5V supply to these motors is provided by separate low drop voltage regulator "5V Servo" which can
 	supply maximum of 800mA current. It is a good practice to move one servo at a time to reduce power surge 
	in the robot's supply lines. Also preferably take ADC readings while servo motor is not moving or stopped
	moving after giving desired position.
*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

//Initialize the ports
void port_init(void)
{
 servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation 
 servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation  
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


//Function to initialize all the peripherals
void init_devices(void)
{
 cli(); //disable all interrupts
 port_init();
 timer1_init();
 sei(); //re-enable interrupts 
}


//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free (void) //makes servo 3 free rotating
{
 OCR1CH = 0x03;
 OCR1CL = 0xFF; //Servo 3 off
} 



//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to initialize ports
void init_ports()
{
 motion_pin_config();
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortARestore = PORTA; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore; 			// setting the command to the port
}



void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}



void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop (void)
{
  motion_set(0x00);
}

void init_devices1 (void) //use this function to initialize all devices
{
 cli(); //disable all interrupts
 init_ports();
 timer5_init();
 sei(); //re-enable interrupts
}

void flip()
	{
		_delay_ms(1000);
		
		for(int i=0;i<22;i++)                               //angle_end
		{
			servo_1(i);
			
			servo_2(i);
			
			servo_3(i);
			
			_delay_ms(7);
			
		}
		
		
		_delay_ms(1000);
		
		for(int i=22;i<58;i++)                               //angle_end
		{
			servo_1(i);
			
			servo_2(i);
			
			servo_3(i);
			
			_delay_ms(7);
			
		}
		
		
		_delay_ms(500);
		servo_1_free();
		servo_2_free();
		servo_3_free();
		
		for(int i=58;i>=0;i--)                               //angle_end
		{
			servo_1(i);
			
			servo_2(i);
			
			servo_3(i);
			_delay_ms(7);
		}
		
		
		_delay_ms(500);
		servo_1_free();
		servo_2_free();
		servo_3_free();
		
		
		_delay_ms(1000);
	
	}

void holdrotate(int n)
{
	_delay_ms(1000);
	for(int i=0;i<22;i++)                               //angle_end
	{
		servo_1(i);
		
		servo_2(i);
		
		servo_3(i);
		
		_delay_ms(7);
		
	}
	
	
	_delay_ms(1000);
	servo_1_free();
	servo_2_free();
	servo_3_free();
	
	if(n<3)
	{
		
	
	soft_right_2(); //Left wheel stationary, Right wheel forward
	
	for(int x=0;x<n;x++)
	_delay_ms(302);
	
	stop();
	}
	else
	{
		
		
		soft_left(); //Left wheel stationary, Right wheel forward
		
		
		_delay_ms(302);
		
		stop();
	}	
	
	
	
	
	_delay_ms(1000);
	
	for(int i=22;i>=0;i--)                               //angle_end
	{
		servo_1(i);
		
		servo_2(i);
		
		servo_3(i);
		_delay_ms(7);
	}
	
	
	_delay_ms(500);
	servo_1_free();
	servo_2_free();
	servo_3_free();
	
}	
	

	

void freerotate(int n)
{	_delay_ms(1000);
	
	if(n<3)
	{
		
	
	soft_right_2();
	for(int a=0;a<n ; a++)
	{
	 //Left wheel stationary, Right wheel backward
	_delay_ms(235);
	}
	
	stop();
	}
	else
	{
		
		
		soft_left();
	
			
			_delay_ms(230);
		
		stop();
	}
		
}

void left (int n)
{freerotate(3);
	flip();
	holdrotate(n);

	freerotate(2);
	flip();
	freerotate(3);
}

void right(int n)
{
	freerotate(1);
	flip();
	holdrotate(1*n);

	freerotate(2);
	flip();
	freerotate(1);
}

void down(int n)
{
	holdrotate(1*n);
}

void up(int n)
{
	flip();
	flip();
	holdrotate(n);
	flip();
	flip();
}

void back(int n)
{
	flip();
	holdrotate(1*n);

	freerotate(2);
	flip();
	freerotate(2);
}

void front(int n)
{
	freerotate(2);
	flip();
	holdrotate(n);

	freerotate(2);
	flip();
}


int main()
{
	
	
	init_devices();
	
	
	
	init_devices1();
	velocity (255, 255); //Set robot velocity here. Smaller the value lesser will be the velocity
						 //Try different values between 0 to 255
						 
	
	flip();
	flip();
	flip();
	flip();
		
	/*for(int i=36;i>=0;i--)                               //angle_end
	{
		servo_1(i);
		
		servo_2(i);
		
		servo_3(i);
		_delay_ms(10);
	}
	
	
	_delay_ms(500);
	servo_1_free();
	servo_2_free();
	servo_3_free();
	*/
	
	
	char str[]="F1 D1 B1 F1 D1 F2 D1 L1 D3 L1 D1 B2 D3 B2 D3 B2 D1 R3 D3 F2 U1 D2 B2 U2 L2 D2 B2 F2 L2 F2 L2\0";	//input for solving present state of cube(to be updated as required)				 
	int n; char ch;
	int i=0;
	while (str[i]!='\0')
	{if (isalpha(str[i]))
		ch=str[i];

		if(isdigit(str[i]))
		{
			n=((int)str[i])-48;

			switch(ch)
			{
				case 'L': left(n); break;
				case 'R': right(n); break;
				case 'U': up(n); break;
				case 'D': down(n); break;
				case 'F': front(n); break;
				case 'B': back(n); break;
				default: break;

			}


		}
		i++;
	}

					 


return 1;
}

