/*
             LUFA Library
     Copyright (C) Dean Camera, 2011.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2011  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/


/** \file
 *
 *  Main source file for the VirtualSerialMouse demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "VirtualSerialMouse.h"

//<-----------------------------------Global Variables-------------------------------->
	
	//Analog Values
	volatile uint16_t gyro_30_x=0, gyro_120_x=0, gyro_30_y=0, gyro_120_y=0;
	volatile uint16_t accel_x=0, accel_y=0, accel_z=0;
	volatile uint16_t left_thumb_x=0, left_thumb_y=0, left_hat_x=0, left_hat_y=0;
	
	//ADC ISR values
	volatile uint8_t conversion_status = 0;
	volatile uint8_t thumb_count = 0;
	
	//digital debounced values (register logged as an unsigned int)
	uint8_t digital_c=0xFF, digital_d=0xFF;
	//type of button or use for each pin
	uint8_t digital_c_type[8] = {0, 0, 0, 0, 0, 2, 0, 0}; // |key|key|key|key|key|mouse|key|key|
	uint8_t digital_d_type[8] = {1, 2, 2, 0, 0, 3, 0, 0}; // |modifier|mouse|mouse|key|key|motion disable|key|key|
	
	double mouse_move_y;
	double mouse_move_hist_y;
	double mouse_move_x;
	double mouse_move_hist_x;
	double temp;
	
	//mouse position
	uint16_t mouse_pos_y = 1080;
	
	//var for mouse or keyboard report sending
	uint8_t mouse_report;
	
	struct common_options *common = &current_options;
	struct profile *prof = &current_profile;
	struct controller *cont_old = &controller_vector;
	struct controller *cont_new = &controller_vector;
	struct serial *debug = &serial_data;

//<-----------------------------------------Struct Declarations----------------------------------------------->
	
/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = 0,

				.DataINEndpointNumber           = CDC_TX_EPNUM,
				.DataINEndpointSize             = CDC_TXRX_EPSIZE,
				.DataINEndpointDoubleBank       = false,

				.DataOUTEndpointNumber          = CDC_RX_EPNUM,
				.DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
				.DataOUTEndpointDoubleBank      = false,

				.NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
				.NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
				.NotificationEndpointDoubleBank = false,
			},
	};

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevHIDReportBuffer[MAX(sizeof(USB_KeyboardReport_Data_t), sizeof(USB_MouseReport_Data_t))];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Device_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = 0,

				.ReportINEndpointNumber       = HID_IN_EPNUM,
				.ReportINEndpointSize         = HID_EPSIZE,
				.ReportINEndpointDoubleBank   = false,

				.PrevReportINBuffer           = PrevHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevHIDReportBuffer),
			},
	};
	

//<-----------------------------------Main entry point------------------------------------------------------>

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();
	sei();

	default_profile(prof);
	default_common_options(common);
	
	mouse_report = 0; //set to keyboard first
	
	initial_calibration();
	
	//set the next ADC conversion
	ADCSRA |= (1<<ADSC);
	conversion_status = 1;
	
	for (;;)
	{
		read_buttons();
		/* Must throw away unused bytes from the host, or it will lock up while waiting for the device */
		CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

		if(conversion_status == 0)
		{
			analog_filter();
			//send_serial_data();
			//debug transmit analog raw data
			//CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
			//gyro_stabilize(); todo
			HID_Device_USBTask(&Device_HID_Interface);
			USB_USBTask();
			
			//set the next ADC conversion
			ADCSRA |= (1<<ADSC);
			conversion_status = 1;
		}
	}
}

//<-------------------------------------Function Declarations------------------------------------------------->
/*
void send_serial_data(void)
{
	//This was to be used with a PC based config tool to change controls
	//this feature was never fully implemented
	debug -> start_byte = 0xAA;			//1
	debug -> gyro_30_x = gyro_30_x;		//2
	debug -> gyro_120_x = gyro_120_x;	//2
	debug -> gyro_30_y = gyro_30_y;		//2
	debug -> gyro_120_y = gyro_120_y;	//2
	debug -> accel_x = accel_x;			//2
	debug -> accel_y = accel_y;			//2
	debug -> accel_z = accel_z;			//2
	debug -> left_thumb_x = left_thumb_x;		//2
	debug -> left_thumb_y = left_thumb_y;		//2
	debug -> left_hat_x = left_hat_x;		//2
	debug -> left_hat_y = left_hat_y;		//2
	debug -> end_byte = 0xD5;		//1
	
	CDC_Device_SendData(&VirtualSerial_CDC_Interface, &serial_data, sizeof(serial_data));
}
*/

/*
void gyro_stabilize(void)
{
	//Test of a method to correct gyro drift but never finished the logic behind it
	
	static uint8_t gyro_30_x_stab = 0;
	static uint8_t gyro_30_y_stab = 0;
	static uint8_t gyro_120_x_stab = 0;
	static uint8_t gyro_120_y_stab = 0;
	
	static uint16_t gyro_30_x_hist = 50000;
	static uint16_t gyro_30_y_hist = 50000;
	static uint16_t gyro_120_x_hist = 50000;
	static uint16_t gyro_120_y_hist = 50000;
	
	if(gyro_30_x_hist == 50000)
	{
		gyro_30_x_hist = gyro_30_x;
		gyro_30_y_hist = gyro_30_y;
		gyro_120_x_hist = gyro_120_x;
		gyro_120_y_hist = gyro_120_y;
	}
	//check if gyro is drifting (moving at a near constant rate)
	//for a reasonably long period of time
	
	if( (gyro_30_x <= gyro_30_x_hist + 2) && (gyro_30_x >= gyro_30_x_hist - 2) )
	{
		gyro_30_x_stab++;
	}else
	{
		gyro_30_x_hist = gyro_30_x;
	}
	
	if( (gyro_30_y <= gyro_30_y_hist + 2) && (gyro_30_y >= gyro_30_y_hist - 2) )
	{
		gyro_30_y_stab++;
	}else
	{
		gyro_30_y_hist = gyro_30_y;
	}
	
	if( (gyro_120_x <= gyro_120_x_hist + 2) && (gyro_120_x >= gyro_120_x_hist - 2) )
	{
		gyro_120_x_stab++;
	}else
	{
		gyro_120_x_hist = gyro_120_x;
	}
	
	if( (gyro_120_y <= gyro_120_y_hist + 2) && (gyro_120_y >= gyro_120_y_hist - 2) )
	{
		gyro_120_y_stab++;
	}else
	{
		gyro_120_y_hist = gyro_120_y;
	}
	*/
	/*
	//Fully non working code here
	if(gyro_30_x_stab == 10)
	{
		if(gyro_30_x_hist < gyro_30_x)
		{
			common -> gyro_30_mid_x += .1;
		}else
		{
			common -> gyro_30_mid_x -= .1;
		}
		gyro_30_x_stab = 0;
	}
	
	if(gyro_30_y_stab == 10)
	{
		if(gyro_30_y_hist < gyro_30_y)
		{
			common -> gyro_30_mid_y += .1;
		}else
		{
			common -> gyro_30_mid_y -= .1;
		}
		gyro_30_y_stab = 0;
	}
	
	if(gyro_120_x_stab == 10)
	{
		if(gyro_120_x_hist < gyro_120_x)
		{
			common -> gyro_120_mid_x += .1;
		}else
		{
			common -> gyro_120_mid_x -= .1;
		}
		gyro_120_x_stab = 0;
	}
	
	if(gyro_120_y_stab == 10)
	{
		if(gyro_120_y_hist < gyro_30_y)
		{
			common -> gyro_120_mid_y += .1;
		}else
		{
			common -> gyro_120_mid_y -= .1;
		}
		gyro_120_y_stab = 0;
	}
	*/
/*
}
*/

void default_profile(struct profile *cur_profile)
{
	//Right now swapping profiles or changing them after compile time is not possible
	//hardcode all controls in here


	//load a default profile 
	//key mapping data
	//left thumb stick
	cur_profile -> left_thumb_foreward = HID_KEYBOARD_SC_W; 		//0x1A
	cur_profile -> left_thumb_back = HID_KEYBOARD_SC_S; 			//0x16
	cur_profile -> left_thumb_left = HID_KEYBOARD_SC_A; 			//0x04
	cur_profile -> left_thumb_right = HID_KEYBOARD_SC_D;			//0x07
	cur_profile -> left_thumb_click = HID_KEYBOARD_MODIFER_LEFTSHIFT;// HID_KEYBOARD_SC_T;
	//left hat (d-pad)
	cur_profile -> left_hat_foreward = HID_KEYBOARD_SC_1_AND_EXCLAMATION;	//0x1E
	cur_profile -> left_hat_back = HID_KEYBOARD_SC_2_AND_AT;				//0x1F
	cur_profile -> left_hat_left = HID_KEYBOARD_SC_3_AND_HASHMARK;			//0x20
	cur_profile -> left_hat_right = HID_KEYBOARD_SC_4_AND_DOLLAR;			//0x21
	//trigger
	cur_profile -> trigger = (1 << 0); //mouse button 1
	//right side buttons (1 being the front one)
	cur_profile -> right_button_1 = (1 << 1); //mouse button 2
	cur_profile -> right_button_2 = HID_KEYBOARD_SC_G;			//0x0A
	cur_profile -> right_button_3 = HID_KEYBOARD_SC_F;			//0x09
	cur_profile -> right_button_4 = 0x00; //disable motion control (0x00)
	//center buttons (right hand usage)
	// | 1 | 2 | 3 |
	// | 4 | 5 | 6 |
	cur_profile -> center_button_1 = HID_KEYBOARD_SC_E;			//0x08
	cur_profile -> center_button_2 = HID_KEYBOARD_SC_J;			//0x0D
	cur_profile -> center_button_3 = HID_KEYBOARD_SC_V;			//0x19
	cur_profile -> center_button_4 = HID_KEYBOARD_SC_K;			//0x0E
	cur_profile -> center_button_5 = HID_KEYBOARD_SC_SPACE;		//0x2C
	cur_profile -> center_button_6 = (1 << 2); //mouse button 3
	//charging lever
	cur_profile -> charging_lever = HID_KEYBOARD_SC_R;			//0x15
	//motion controls
	//cur_profile -> motion_stab = ;//mouse button 3
	cur_profile -> motion_lean_left = HID_KEYBOARD_SC_Q;		//0x14
	cur_profile -> motion_lean_right = HID_KEYBOARD_SC_E;		//0x08
	
	//Advanced settings
	//mouse sensitivity
	cur_profile -> mouse_sensitivity_x = 20;
	cur_profile -> mouse_sensitivity_y = 20;
	//mouse invert
	cur_profile -> mouse_invert_x = 0;
	cur_profile -> mouse_invert_y = 1;
	//motion control enable settings
	cur_profile -> motion_enable_stab = 0;
	cur_profile -> motion_enable_lean = 0;
	//mouse end points
	cur_profile -> mouse_endpoint_min_y = 0;
	cur_profile -> mouse_endpoint_max_y = 2160;
	//enable option for mouse end points
	cur_profile -> mouse_endpoint_enable = 1;
	//deadzone settings
	cur_profile -> mouse_deadzone_x = 50;
	cur_profile -> mouse_deadzone_y = 50;
	cur_profile -> left_thumb_deadzone_x = 350;
	cur_profile -> left_thumb_deadzone_y = 350;
	cur_profile -> left_hat_deadzone_x = 350;
	cur_profile -> left_hat_deadzone_y = 350;
}


void default_common_options(struct common_options *common)
{
	//Calibration data default values
	//mid values
	common->gyro_30_mid_x = 310;
	common->gyro_30_mid_y = 310;
	common->gyro_120_mid_x = 315;
	common->gyro_120_mid_y = 315;
	//lower end point values
	common->accel_min_x = 1024;
	common->accel_min_y = 1024;
	common->accel_min_z = 1024;
	common->left_thumb_min_x = 500;
	common->left_thumb_min_y = 500;
	common->left_hat_min_x = 500;
	common->left_hat_min_y = 500;
	//upper end point values
	common->accel_max_x = 0;
	common->accel_max_y = 0;
	common->accel_max_z = 0;
	common->left_thumb_max_x = 500;
	common->left_thumb_max_y = 500;
	common->left_hat_max_x = 500;
	common->left_hat_max_y = 500;
}


/** Configures the board hardware and chip peripherals */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	//This is set up for a Teensy 2.0++ (AT90USB1286) running at 16MHz
	//if using a different board these may need to be changed
	
	DDRD = 0x00; 		//configure port D as inputs with pullup resistors
	PORTD = 0xFF;
	DDRC = 0x00; 		//configure port C as inputs with pullup resistors
	PORTC = 0xFF;
	DDRB = 0x00; 		//configure port B as inputs with pullup resistors
	PORTB = 0xFF;
	DDRA = 0x00; 		//configure port A as inputs with pullup resistors
	PORTA = 0xFF;
	DDRE = 0x23; 	//set bit 0, 1, 6 as outputs
	PORTE = 0x20; 	//set bit 6 to high
	
	//setup the ADC
	ADMUX = 0x00; //set to external Vref, channel 0
	//ADMUX bits 0, 1, 2 set the ADC channel
	//ADEN - enable ADC, ADSC - start conversion
	//enables the ADC and the interrupt, set the division factor to 128
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	DIDR0 = 0xFF;  		// all analog pins (F0-F7) have digital input disabled
	
	/* Hardware Initialization */
	USB_Init();
}

void analog_filter(void)
{
	//There are two gyros one with a sensitivity of 30 degrees per second
	//and one with 120 degrees per second
	//This allows for higher precision at low speed and still permits higher
	//speed when the 30 degree gyro has saturated
	
	//Note: these are analog output gyros. at some point they will be switched to digital ones
	
	//gyro_30_x
	gyro_30_x = fmin(fmax(gyro_30_x, 0), 2 * common -> gyro_30_mid_x);
	//gyro_30_y
	gyro_30_y = fmin(fmax(gyro_30_y, 0), 2 * common -> gyro_30_mid_y);
	//gyro_120_x
	gyro_120_x = fmin(fmax(gyro_120_x, 0), 2 * common -> gyro_120_mid_x);
	//gyro_120_y
	gyro_120_y = fmin(fmax(gyro_120_y, 0), 2 * common -> gyro_120_mid_y);
	//accel_min_x
	
	if(accel_x < common -> accel_min_x)
		{common -> accel_min_x = accel_x;}
	if(accel_x > common -> accel_min_x)
		{common -> accel_min_x = accel_x;}
	//accel_min_y
	if(accel_y < common -> accel_min_y)
		{common -> accel_min_y = accel_y;}
	if(accel_y > common -> accel_min_y)
		{common -> accel_min_y = accel_y;}
	//accel_min_z
	if(accel_z < common -> accel_min_z)
		{common -> accel_min_z = accel_z;}
	if(accel_z > common -> accel_min_z)
		{common -> accel_min_z = accel_z;}
	//left_thumb_min_x
	if(left_thumb_x < common -> left_thumb_min_x)
		{common -> left_thumb_min_x = left_thumb_x;}
	if(left_thumb_x > common -> left_thumb_min_x)
		{common -> left_thumb_min_x = left_thumb_x;}
	//left_thumb_min_y
	if(left_thumb_y < common -> left_thumb_min_y)
		{common -> left_thumb_min_y = left_thumb_y;}
	if(left_thumb_y > common -> left_thumb_min_y)
		{common -> left_thumb_min_y = left_thumb_y;}
	//left_hat_min_x
	if(left_hat_x < common -> left_hat_min_x)
		{common -> left_hat_min_x = left_hat_x;}
	if(left_hat_x > common -> left_hat_min_x)
		{common -> left_hat_min_x = left_hat_x;}
	//left_hat_min_y
	if(left_hat_y < common -> left_hat_min_y)
		{common -> left_hat_min_y = left_hat_y;}
	if(left_hat_y > common -> left_hat_min_y)
		{common -> left_hat_min_y = left_hat_y;}
	
}

void initial_calibration(void)
{
	uint16_t i;
	for(i=0;i<1000;i++)
	{
		//set the next ADC conversion
		ADCSRA |= (1<<ADSC);
		conversion_status = 1;
		
		while(conversion_status != 0)
		{
		//average 10000 samples for each analog value to find the mid point
		//note that on power on the controller must be left perfectly still
		//and level
		common -> gyro_30_mid_x = (common -> gyro_30_mid_x + gyro_30_x) / 2;
		common -> gyro_30_mid_y = (common -> gyro_30_mid_y + gyro_30_y) / 2;
		common -> gyro_120_mid_x = (common -> gyro_120_mid_x + gyro_120_x) / 2;
		common -> gyro_120_mid_y = (common -> gyro_120_mid_y + gyro_120_y) / 2;
		}
	}
}

void update_controller_vector(void)
{
	//move the latest vector into the old vector
	cont_old -> x_vector = cont_new -> x_vector;
	cont_old -> y_vector = cont_new -> y_vector;
	cont_old -> z_vector = cont_new -> z_vector;
	cont_old -> x_rot_vector = cont_new -> x_rot_vector;
	cont_old -> y_rot_vector = cont_new -> y_rot_vector;
	cont_old -> z_rot_vector = cont_new -> z_rot_vector;
	//update the new vector based on analog values
}


ISR(ADC_vect)
{	
	//first entry assumes ADMUX = 0x00;
	//PORTE B0, B1, B6 all == 0
	if(conversion_status == 1) //left_120_x
	{
		gyro_120_x = (gyro_120_x * gyro_filter) + ((1 - gyro_filter) * (ADCL | (ADCH << 8)));
		ADMUX = 0x01; 						
		conversion_status++;
		ADCSRA |= (1<<ADSC); 				//enable the next conversion
	}else if(conversion_status == 2) //gyro_120_y
	{
		gyro_120_y = (gyro_120_y * gyro_filter) + ((1 - gyro_filter) * ( ADCL | (ADCH << 8)));	//read the second input
		ADMUX = 0x02;
		conversion_status++;
		ADCSRA |= (1<<ADSC); 
	}else if(conversion_status == 3) //accel_x
	{
		accel_x = (accel_x * accel_filter) + ((1 - accel_filter) * ( ADCL | (ADCH << 8)));		//read the third input
		ADMUX = 0x03; 						//set to fourth input port
		conversion_status++;
		ADCSRA |= (1<<ADSC); 				//enable the next conversion
	}else if(conversion_status == 4) //accel_y
	{
		accel_y = (accel_y * accel_filter) + ((1 - accel_filter) * ( ADCL | (ADCH << 8)));		//read the fourth input
		ADMUX = 0x04;
		conversion_status++;
		ADCSRA |= (1<<ADSC); 				//enable the next conversion
	}else if(conversion_status == 5) //accel_z
	{
		accel_z = (accel_z * accel_filter) + ((1 - accel_filter) * ( ADCL | (ADCH << 8)));		//read the fifth input
		ADMUX = 0x05; 						//set to sixth input port
		conversion_status++;
		ADCSRA |= (1<<ADSC); 				//enable the next conversion
	}else if(conversion_status == 6) //gyro_30_y
	{
		gyro_30_y = (gyro_30_y * gyro_filter) + ((1 - gyro_filter) * (ADCL | (ADCH << 8)));	//read the sixth input
		ADMUX = 0x06; 						//set to seventh input port
		conversion_status++;
		ADCSRA |= (1<<ADSC); 				//enable the next conversion
	}else if(conversion_status == 7) //gyro_30_x
	{
		gyro_30_x = (gyro_30_x * gyro_filter) + ((1 - gyro_filter) * (ADCL | (ADCH << 8)));	//read the seventh input
		ADMUX = 0x07; 						//set to eighth input port
		conversion_status++;
		ADCSRA |= (1<<ADSC);				//enable the next conversion
	}else if(conversion_status == 8) //accel_z
	{
		if(thumb_count == 0)
		{
			left_hat_y = (left_hat_y * thumbstick_filter) + ((1 - thumbstick_filter) * (ADCL | (ADCH << 8)));
			//PORTE |= (1 << 0); //set 101
			PORTE = 0b00000001;
			thumb_count++;
		}
		else if(thumb_count == 1)
		{
			left_hat_x = (left_hat_x * thumbstick_filter) + ((1 - thumbstick_filter) * (ADCL | (ADCH << 8)));
			//PORTE &= ~(1 << 0);
			//PORTE |= (1 << 1); //set 110
			PORTE = 0b00100010;
			thumb_count++;
		}
		else if(thumb_count == 2)
		{
			left_thumb_x = (left_thumb_x * thumbstick_filter) + ((1 - thumbstick_filter) * (ADCL | (ADCH << 8)));
			//PORTE |= (1 << 0); //set 111
			PORTE = 0b00100011;
			thumb_count++;
		}
		else if(thumb_count == 3)
		{
			left_thumb_y = (left_thumb_y * thumbstick_filter) + ((1 - thumbstick_filter) * (ADCL | (ADCH << 8)));
			//PORTE &= ~(1 << 1) & ~(1 << 0); //set 100
			PORTE = 0b00000000;
			thumb_count = 0;
		}
		ADMUX = 0x00;
		conversion_status = 0;
	}
}

//<-----------------------------------Read and debounce button input---------------------------------->
void read_buttons(void)
{
	static uint8_t flag_d[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	static uint8_t old_d[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t i;
	static uint8_t flag_c[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	static uint8_t old_c[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	for(i=0;i<8;i++)
	{
		//PIND
		//analog style filter for the button inputs
		old_d[i] = old_d[i] * .75;
		if(bit_is_clear(PIND, i)){old_d[i] = old_d[i] + 0x3F;}
		//trigger to switch when the analog value gets high or low
		if((old_d[i] > 0xF0)&&(flag_d[i]==0)){flag_d[i]=1; digital_d &= ~(1 << i);}
		if((old_d[i] < 0x0F)&&(flag_d[i]==1)){flag_d[i]=0; digital_d |= (1 << i);}
		//PINC
		//analog style filter for the button inputs
		old_c[i] = old_c[i] * .75;
		if(bit_is_clear(PINC, i)){old_c[i] = old_c[i] + 0x3F;}
		//trigger to switch when the analog value gets high or low
		if((old_c[i] > 0xF0)&&(flag_c[i]==0)){flag_c[i]=1; digital_c &= ~(1 << i);}
		if((old_c[i] < 0x0F)&&(flag_c[i]==1)){flag_c[i]=0; digital_c |= (1 << i);}
	}
	//debounced values are stored in digital_d and digital_c
}


//<-----------------------------------USB HID reports------------------------------------------------------------>
/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent
 *
 *  \return Boolean true to force the sending of the report, false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	//check if the current set report is mouse or keyboard
	if (mouse_report==0)
	{
//<----------------------------------Keyboard Code------------------------------------------------->
		USB_KeyboardReport_Data_t* KeyboardReport = (USB_KeyboardReport_Data_t*)ReportData;
		
		//KeyboardReport->Modifier = HID_KEYBOARD_MODIFER_LEFTSHIFT;
		//KeyboardReport->KeyCode[0] = HID_KEYBOARD_SC_A; //keycode values 0-5
		uint8_t mask = 1;
		uint8_t i, j;
		
		//set all keyboard keys to 0 (unpressed)
		KeyboardReport->KeyCode[0] = 0;
		KeyboardReport->KeyCode[1] = 0;
		KeyboardReport->KeyCode[2] = 0;
		KeyboardReport->KeyCode[3] = 0;
		KeyboardReport->KeyCode[4] = 0;
		KeyboardReport->KeyCode[5] = 0;
		//set the modifier keys to unpressed
		KeyboardReport->Modifier = 0;
		
		//thumbstick and hat switch button handling
		//button is considered pressed when it exceeds the deadzone amount away from center
		//center is defined as exactly in between the upper and lower extremes
		uint8_t thumbs = 0xFF;
	
		if(left_thumb_x > 600)
		{
			thumbs &= ~(1 << 0);
		}
		if(left_thumb_x < 400)
		{
			thumbs &= ~(1 << 1);
		}
		if(left_thumb_y > 600)
		{
			thumbs &= ~(1 << 2);
		}
		if(left_thumb_y < 400)
		{
			thumbs &= ~(1 << 3);
		}
		
		if(left_hat_x > 700)
		{
	//		thumbs &= ~(1 << 4);
		}
		if(left_hat_x < 300)
		{
	//		thumbs &= ~(1 << 5);
		}
		if(left_hat_y > 700)
		{
	//		thumbs &= ~(1 << 6);
		}
		if(left_hat_y < 300)
		{
	//		thumbs &= ~(1 << 7);
		}
		
	/*
		//thumbstick
		//if pushed left beyond the middle plus deadzone
		if(left_thumb_x > ( (common -> left_thumb_max_x - common -> left_thumb_min_x) / 2
		+ common -> left_thumb_min_x + prof -> left_thumb_deadzone_x) )
		{
			thumbs &= ~(1 << 0);
		} //else if pushed right beyond the middle minus deadzone
		else if(left_thumb_x > ( (common -> left_thumb_max_x - common -> left_thumb_min_x) / 2
		+ common -> left_thumb_min_x - prof -> left_thumb_deadzone_x) )
		{
			thumbs &= ~(1 << 1);
		}
		//if pushed foreward beyond the middle plus deadzone
		if(left_thumb_y > ( (common -> left_thumb_max_y - common -> left_thumb_min_y) / 2
		+ common -> left_thumb_min_y + prof -> left_thumb_deadzone_y) )
		{
			thumbs &= ~(1 << 2);
		} 
		//else if pushed back beyond the middle minus deadzone
		else if(left_thumb_y > ( (common -> left_thumb_max_y - common -> left_thumb_min_y) / 2
		+ common -> left_thumb_min_y - prof -> left_thumb_deadzone_y) )
		{
			thumbs &= ~(1 << 3);
		}
	*/
	/*
		//hat switch controls
		//if pushed left beyond the middle plus deadzone
		if(left_hat_x > ( (common -> left_hat_max_x - common -> left_hat_min_x) / 2
		+ common -> left_hat_min_x + prof -> left_hat_deadzone_x) )
		{
			thumbs &= ~(1 << 4);
		} //else if pushed right beyond the middle minus deadzone
		else if(left_hat_x > ( (common -> left_hat_max_x - common -> left_hat_min_x) / 2
		+ common -> left_hat_min_x - prof -> left_hat_deadzone_x) )
		{
			thumbs &= ~(1 << 5);
		}
		//if pushed foreward beyond the middle plus deadzone
		if(left_hat_y > ( (common -> left_hat_max_y - common -> left_hat_min_y) / 2
		+ common -> left_hat_min_y + prof -> left_hat_deadzone_y) )
		{
			thumbs &= ~(1 << 6);
		} 
		//else if pushed back beyond the middle minus deadzone
		else if(left_hat_y > ( (common -> left_hat_max_y - common -> left_hat_min_y) / 2
		+ common -> left_hat_min_y - prof -> left_hat_deadzone_y) )
		{
			thumbs &= ~(1 << 7);
		}
		//currently can only be of type key no modifiers or mouse buttons allowed
		//todo add other types
	*/
		for(i=0; i<8; i++)
		{
			if ((thumbs & mask) == 0) //checks all the bits of thumbs
			{
				for(j=0;j<6;j++)
				{
					if(KeyboardReport->KeyCode[j]==0)
					{
						if(i==0){KeyboardReport->KeyCode[j] = LTL;}
						if(i==1){KeyboardReport->KeyCode[j] = LTR;}
						if(i==2){KeyboardReport->KeyCode[j] = LTF;}
						if(i==3){KeyboardReport->KeyCode[j] = LTB;}
						if(i==4){KeyboardReport->KeyCode[j] = LHL;}
						if(i==5){KeyboardReport->KeyCode[j] = LHR;}
						if(i==6){KeyboardReport->KeyCode[j] = LHF;}
						if(i==7){KeyboardReport->KeyCode[j] = LHB;}
						break;
					}
				}
			}
			//shift the mask bit to check the next pin
			mask = mask << 1;
		}
		
		
		mask = 1; //reset the mask
		
		// button states are held in digital_c and digital_d
		// button type is stored in digital_c_type[8] and digital_d_type[8]
		// type==0 is regular key, type==1 is modifer key, type==2 is mouse button, type 3 is motion disable
		// types 2 and 3 are handled in the mouse section
		
		for (i=0; i<8; i++) //main loop for all the button presses
		{
			if ((digital_c & mask) == 0) //checks all the bits of digital_c
			{
				if(digital_c_type[i] == 0) //if type is regular key
				{
					for(j=0; j<6; j++) //loop through the keyboard_keys for an unused one
					{
						//check if an unused key place holder is found
						if(KeyboardReport->KeyCode[j]==0)
						{
							//set the cooresponding key (based on i) to the empty spot (j)
							#ifdef PCB0					
								if(i==0){KeyboardReport->KeyCode[j] = PCB0;}
							#endif
							#ifdef PCB1
								if(i==1){KeyboardReport->KeyCode[j] = PCB1;}
							#endif
							#ifdef PCB2	
								if(i==2){KeyboardReport->KeyCode[j] = PCB2;}
							#endif
							#ifdef PCB3	
								if(i==3){KeyboardReport->KeyCode[j] = PCB3;}
							#endif
							#ifdef PCB4	
								if(i==4){KeyboardReport->KeyCode[j] = PCB4;}
							#endif
							#ifdef PCB5	
								if(i==5){KeyboardReport->KeyCode[j] = PCB5;}
							#endif
							#ifdef PCB6	
								if(i==6){KeyboardReport->KeyCode[j] = PCB6;}
							#endif
							#ifdef PCB7	
								if(i==7){KeyboardReport->KeyCode[j] = PCB7;}
							#endif
							break;
						}
					}
				}
				else if(digital_c_type[i] == 1) //if type is modifier key
				{
					//set the cooresponding key (based on i) to the modifier
					#ifdef PCB0
						if(i==0){KeyboardReport->Modifier |= PCB0;}
					#endif
					#ifdef PCB1
						if(i==1){KeyboardReport->Modifier |= PCB1;}
					#endif
					#ifdef PCB2
						if(i==2){KeyboardReport->Modifier |= PCB2;}
					#endif
					#ifdef PCB3
						if(i==3){KeyboardReport->Modifier |= PCB3;}
					#endif
					#ifdef PCB4
						if(i==4){KeyboardReport->Modifier |= PCB4;}
					#endif
					#ifdef PCB5
						if(i==5){KeyboardReport->Modifier |= PCB5;}
					#endif
					#ifdef PCB6
						if(i==6){KeyboardReport->Modifier |= PCB6;}
					#endif
					#ifdef PCB7
						if(i==7){KeyboardReport->Modifier |= PCB7;}
					#endif
				}
			}//digital_c
			
			//special case pin 3 is active high
			if ( (((digital_d & mask) == 0) && i != 3) || (((digital_d & mask) != 0) && i == 3) ) //checks all the pins of port D for a button press
			{
				if(digital_d_type[i] == 0) //if type is regular key
				{
					for(j=0; j<6; j++) //loop through the keyboard_keys for an unused one
					{
						//check if an unused key placee holder is found
						if(KeyboardReport->KeyCode[j]==0)
						{
							//set the coorisponding key (based on i) to the empty (j)
							#ifdef PDB0					
								if(i==0){KeyboardReport->KeyCode[j] = PDB0;}
							#endif
							#ifdef PDB1
								if(i==1){KeyboardReport->KeyCode[j] = PDB1;}
							#endif
							#ifdef PDB2	
								if(i==2){KeyboardReport->KeyCode[j] = PDB2;}
							#endif
							#ifdef PDB3	
								if(i==3){KeyboardReport->KeyCode[j] = PDB3;}
							#endif
							#ifdef PDB4	
								if(i==4){KeyboardReport->KeyCode[j] = PDB4;}
							#endif
							#ifdef PDB5	
								if(i==5){KeyboardReport->KeyCode[j] = PDB5;}
							#endif
							#ifdef PDB6	
								if(i==6){KeyboardReport->KeyCode[j] = PDB6;}
							#endif
							#ifdef PDB7	
								if(i==7){KeyboardReport->KeyCode[j] = PDB7;}
							#endif
							break;
						}
					}
				}
				else if(digital_d_type[i] == 1) //if type is modifier key
				{
					//set the cooresponding key (based on i) to the modifier
					#ifdef PDB0
						if(i==0){KeyboardReport->Modifier |= PDB0;}
					#endif
					#ifdef PDB1
						if(i==1){KeyboardReport->Modifier |= PDB1;}
					#endif
					#ifdef PDB2
						if(i==2){KeyboardReport->Modifier |= PDB2;}
					#endif
					#ifdef PDB3
						if(i==3){KeyboardReport->Modifier |= PDB3;}
					#endif
					#ifdef PDB4
						if(i==4){KeyboardReport->Modifier |= PDB4;}
					#endif
					#ifdef PDB5
						if(i==5){KeyboardReport->Modifier |= PDB5;}
					#endif
					#ifdef PDB6
						if(i==6){KeyboardReport->Modifier |= PDB6;}
					#endif
					#ifdef PDB7
						if(i==7){KeyboardReport->Modifier |= PDB7;}
					#endif
				}
			}//digital_d
			
			//shift the mask bit to check the next pin
			mask = mask << 1;
		}//main button loop
		
		*ReportID   = HID_REPORTID_KeyboardReport;
		*ReportSize = sizeof(USB_KeyboardReport_Data_t);
		mouse_report=1;
	}else
	{
//<----------------------------------Mouse Code------------------------------------------------------>
		USB_MouseReport_Data_t* MouseReport = (USB_MouseReport_Data_t*)ReportData;
		//set mouse movement and button presses to nothing
		MouseReport->Button = 0; //button 1 (1 << 0), button 2 (1 << 1), button 3 (1 << 2) 
		MouseReport->X = 0; //positive is right
		MouseReport->Y = 0; //positive is down
		uint8_t motion_enable = 1;
		uint8_t i, mask =1;
		
		//MouseReport->Button = (1<<0) | (1<<1);
		
		//read the buttons if set for being a mouse button or motion disable button
		for (i=0; i<8; i++) //main loop for all the button presses
		{
			if ((digital_c & mask) == 0) //checks all the bits of digital_c
			{
				if(digital_c_type[i] == 2) //if type is mouse button
				{
					//set the cooresponding key (based on i)
					#ifdef PCB0
						if(i==0){MouseReport->Button |= PCB0;}
					#endif
					#ifdef PCB1
						if(i==1){MouseReport->Button |= PCB1;}
					#endif
					#ifdef PCB2
						if(i==2){MouseReport->Button |= PCB2;}
					#endif
					#ifdef PCB3
						if(i==3){MouseReport->Button |= PCB3;}
					#endif
					#ifdef PCB4
						if(i==4){MouseReport->Button |= PCB4;}
					#endif
					#ifdef PCB5
						if(i==5){MouseReport->Button |= PCB5;}
					#endif
					#ifdef PCB6
						if(i==6){MouseReport->Button |= PCB6;}
					#endif
					#ifdef PCB7
						if(i==7){MouseReport->Button |= PCB7;}
					#endif
				}
				else if(digital_c_type[i] == 3) //if type is a motion disable button
				{
					//set the cooresponding key (based on i) to the modifier
					#ifdef PCB0
						if(i==0){motion_enable = PCB0;}
					#endif
					#ifdef PCB1
						if(i==1){motion_enable = PCB1;}
					#endif
					#ifdef PCB2
						if(i==2){motion_enable = PCB2;}
					#endif
					#ifdef PCB3
						if(i==3){motion_enable = PCB3;}
					#endif
					#ifdef PCB4
						if(i==4){motion_enable = PCB4;}
					#endif
					#ifdef PCB5
						if(i==5){motion_enable = PCB5;}
					#endif
					#ifdef PCB6
						if(i==6){motion_enable = PCB6;}
					#endif
					#ifdef PCB7
						if(i==7){motion_enable = PCB7;}
					#endif
				}
			}//digital_c
			
			if ((digital_d & mask) == 0) //checks all the pins of port D for a button press
			{
				if(digital_d_type[i] == 2) //if type is regular key
				{
					//set the cooresponding key (based on i) to the empty (j)
					#ifdef PDB0
						if(i==0){MouseReport->Button |= PDB0;}
					#endif
					#ifdef PDB1
						if(i==1){MouseReport->Button |= PDB1;}
					#endif
					#ifdef PDB2
						if(i==2){MouseReport->Button |= PDB2;}
					#endif
					#ifdef PDB3
						if(i==3){MouseReport->Button |= PDB3;}
					#endif
					#ifdef PDB4
						if(i==4){MouseReport->Button |= PDB4;}
					#endif
					#ifdef PDB5
						if(i==5){MouseReport->Button |= PDB5;}
					#endif
					#ifdef PDB6
						if(i==6){MouseReport->Button |= PDB6;}
					#endif
					#ifdef PDB7
						if(i==7){MouseReport->Button |= PDB7;}
					#endif
				}
				else if(digital_d_type[i] == 3) //if type is modifier key
				{
					//set the cooresponding key (based on i) to the modifier
					#ifdef PDB0
						if(i==0){motion_enable = PDB0;}
					#endif
					#ifdef PDB1
						if(i==1){motion_enable = PDB1;}
					#endif
					#ifdef PDB2
						if(i==2){motion_enable = PDB2;}
					#endif
					#ifdef PDB3
						if(i==3){motion_enable = PDB3;}
					#endif
					#ifdef PDB4
						if(i==4){motion_enable = PDB4;}
					#endif
					#ifdef PDB5
						if(i==5){motion_enable = PDB5;}
					#endif
					#ifdef PDB6
						if(i==6){motion_enable = PDB6;}
					#endif
					#ifdef PDB7
						if(i==7){motion_enable = PDB7;}
					#endif
				}
			}//digital_d
			//shift the mask bit to check the next pin
			mask = mask << 1;
		}//main button loop
		
		mouse_move_vertical();
		mouse_move_horizontal();
		if(motion_enable == 1)
		{	
			mouse_pos_y += (int)mouse_move_y;
			MouseReport->Y = (int)mouse_move_y;
			MouseReport->X = (int)mouse_move_x;
		}
		
		*ReportID   = HID_REPORTID_MouseReport;
		*ReportSize = sizeof(USB_MouseReport_Data_t);
		mouse_report=0;
	}
	return true; //force the report to be sent
}


void mouse_move_vertical(void){
	//look up (gyro value higher then middle)
	//look down (gyro value lower then middle)
	if(gyro_30_y > gyro_end_point && gyro_30_y < 2*common -> gyro_30_mid_y - gyro_end_point)
	{	//use precise gyro for low movement
		mouse_move_y = -((gyro_30_y - common -> gyro_30_mid_y)/(4.0 * (prof -> mouse_sensitivity_y)));
	}else
	{
		mouse_move_y = -((gyro_120_y - common -> gyro_120_mid_y)/(prof -> mouse_sensitivity_y));
	}
	mouse_move_y += mouse_move_hist_y; //add in what was left last time
	mouse_move_hist_y = modf(mouse_move_y, &temp); //remember any partial movement
	if(mouse_move_y >= 0)
	{
		mouse_move_hist_y += fmax(mouse_move_y - 126, 0);
	}else
	{
		mouse_move_hist_y += fmin(mouse_move_y + 126, 0);
	}
	mouse_move_y = fmin(fmax(temp, -126), 126); //constrain mouse value from -126 -> 126
}


void mouse_move_horizontal(void){
	//look left (gyro value higher then middle)
	//look right (gyro value lower then middle)
	if(gyro_30_x > gyro_end_point && gyro_30_x < 2*common -> gyro_30_mid_x - gyro_end_point)
	{	//use precise gyro for low movement
		mouse_move_x = -((gyro_30_x - common -> gyro_30_mid_x)/(4.0 * (prof -> mouse_sensitivity_x)));
	}else
	{
		mouse_move_x = -((gyro_120_x - common -> gyro_120_mid_x)/(prof -> mouse_sensitivity_x));
	}
	
	mouse_move_x += mouse_move_hist_x; //add in what was left last time
	mouse_move_hist_x = modf (mouse_move_x , &temp); //remember any partial movement
	if(mouse_move_x >= 0)
	{
		mouse_move_hist_x += fmax(mouse_move_x - 126, 0);
	}else
	{
		mouse_move_hist_x += fmin(mouse_move_x + 126, 0);
	}
	mouse_move_x = fmin(fmax(temp, -126), 126); //constrain mouse value from -126 -> 126
}


/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	//unused
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	//unused
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Device_HID_Interface);
	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	USB_Device_EnableSOFEvents();
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
	HID_Device_ProcessControlRequest(&Device_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Device_HID_Interface);
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the created report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	// Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
}
