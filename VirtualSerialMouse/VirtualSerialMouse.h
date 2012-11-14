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
 *  Header file for VirtualSerialMouse.c.
 */

#ifndef _VIRTUALSERIAL_MOUSE_H_
#define _VIRTUALSERIAL_MOUSE_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/power.h>
		#include <avr/interrupt.h>
		#include <stdbool.h>
//		#include <string.h>
		#include "math.h"
		#include <avr/sleep.h>
		#include <stdio.h>
		#include <avr/eeprom.h>
		#include "Descriptors.h"
		#include <LUFA/Version.h>
		#include <LUFA/Drivers/USB/USB.h>
	
	/* Analog config */
	/*
		Analog Pin Configuration
		F0 - Gyro_30_X -> Gyro_120_X
		F1 - Gyro_30_Y -> Gyro_120_Y
		F2 - Accel_X
		F3 - Accel_Y
		F4 - Accel_Z
		F5 - Gyro_120_X -> Gyro_30_Y
		F6 - Gyro_120_Y -> Gyro_30_X
		F7 - Mux for thumbsticks (mux control E6,E1,E0)
			000 - Left_Thumb_X
			001 - Left_Thumb_Y
			010 - Left_Hat_x
			011 - Left_Hat_Y
			100 - unused
			101 - unused
			110 - unused
			111 - unused
	*/
	
	/* Defines */
		#define PCB0 prof -> center_button_1
		#define PCB1 prof -> center_button_2
		#define PCB2 prof -> center_button_3
		#define PCB3 prof -> center_button_4
		#define PCB4 prof -> center_button_5
		#define PCB5 prof -> center_button_6
		#define PCB6 prof -> charging_lever
//		#define PCB7 unused

		#define PDB0 prof -> left_thumb_click
		#define PDB1 prof -> trigger
		#define PDB2 prof -> right_button_1
		#define PDB3 prof -> right_button_3
		#define PDB4 prof -> right_button_2
		#define PDB5 prof -> right_button_4
//		#define PDB6 
//		#define PDB7 

		#define LTF prof -> left_thumb_foreward //left thumb fore
		#define LTB prof -> left_thumb_back //left thumn back
		#define LTR prof -> left_thumb_right //left thumb right
		#define LTL prof -> left_thumb_left //left thumn left
		
		#define LHF prof -> left_hat_foreward //left hat fore
		#define LHB prof -> left_hat_back //left hat back
		#define LHR prof -> left_hat_right //left hat up
		#define LHL prof -> left_hat_left //left hat down


		#define gyro_filter .6
		#define accel_filter .6
		#define thumbstick_filter .6
		#define gyro_end_point 20
		
	/* structs */
	struct common_options
	{	
		//Calibration data (common to all profiles)
		//mid values
		double gyro_30_mid_x;
		double gyro_30_mid_y;
		double gyro_120_mid_x;
		double gyro_120_mid_y;
		//lower end point values
		uint16_t accel_min_x;
		uint16_t accel_min_y;
		uint16_t accel_min_z;
		uint16_t left_thumb_min_x;
		uint16_t left_thumb_min_y;
		uint16_t left_hat_min_x;
		uint16_t left_hat_min_y;
		//upper end point values
		uint16_t accel_max_x;
		uint16_t accel_max_y;
		uint16_t accel_max_z;
		uint16_t left_thumb_max_x;
		uint16_t left_thumb_max_y;
		uint16_t left_hat_max_x;
		uint16_t left_hat_max_y;
	} current_options;

	struct profile
	{
		//profile data that is specific to each profile
		//The last used profile will be loaded by default
		
		//key mapping data
		//left thumb stick
		uint8_t left_thumb_foreward;
		uint8_t left_thumb_back;
		uint8_t left_thumb_left;
		uint8_t left_thumb_right;
		uint8_t left_thumb_click;
		//left hat (d-pad)
		uint8_t left_hat_foreward;
		uint8_t left_hat_back;
		uint8_t left_hat_left;
		uint8_t left_hat_right;
		//trigger
		uint8_t trigger;
		//right side buttons (1 being the front one)
		uint8_t right_button_1;
		uint8_t right_button_2;
		uint8_t right_button_3;
		uint8_t right_button_4;
		//center buttons (right hand usage)
		// | 1 | 2 | 3 |
		// | 4 | 5 | 6 |
		uint8_t center_button_1;
		uint8_t center_button_2;
		uint8_t center_button_3;
		uint8_t center_button_4;
		uint8_t center_button_5;
		uint8_t center_button_6;
		//charging lever
		uint8_t charging_lever;
		//motion controls
		uint8_t motion_stab;
		uint8_t motion_lean_left;
		uint8_t motion_lean_right;
		
		//Advanced settings
		//mouse sensitivity
		uint16_t mouse_sensitivity_x;
		uint16_t mouse_sensitivity_y;
		//mouse invert
		uint8_t mouse_invert_x;
		uint8_t mouse_invert_y;
		//motion control enable settings
		uint8_t motion_enable_stab;
		uint8_t motion_enable_lean;
		//mouse end points
		uint16_t mouse_endpoint_min_y;
		uint16_t mouse_endpoint_max_y;
		//enable option for mouse end points
		uint8_t mouse_endpoint_enable;
		//deadzone settings
		uint8_t mouse_deadzone_x;
		uint8_t mouse_deadzone_y;
		uint16_t left_thumb_deadzone_x;
		uint16_t left_thumb_deadzone_y;
		uint16_t left_hat_deadzone_x;
		uint16_t left_hat_deadzone_y;
	} current_profile;
	
	struct controller
	{
		//create an angle and direction vectors
		double x_vector;
		double y_vector;
		double z_vector;
		double x_rot_vector;
		double y_rot_vector;
		double z_rot_vector;
	} controller_vector;
	
	struct serial
	{
		uint8_t start_byte;
		uint16_t gyro_30_x;
		uint16_t gyro_120_x;
		uint16_t gyro_30_y;
		uint16_t gyro_120_y;
		uint16_t accel_x;
		uint16_t accel_y;
		uint16_t accel_z;
		uint16_t left_thumb_x;
		uint16_t left_thumb_y;
		uint16_t left_hat_x;
		uint16_t left_hat_y;
		uint8_t end_byte;
	} serial_data;
	
	
	/* Macros: */


	/* Function Prototypes: */
		void SetupHardware(void);
		void default_profile(struct profile *cur_profile);
		void default_common_options(struct common_options *common);
		void read_buttons(void);
		void initial_calibration(void);
		void analog_filter(void);
		void update_controller_vector(void);
		void send_serial_data(void);
		void mouse_move_vertical(void);
		void mouse_move_horizontal(void);
		void gyro_stabilize(void);

		void EVENT_USB_Device_Connect(void);
		void EVENT_USB_Device_Disconnect(void);
		void EVENT_USB_Device_ConfigurationChanged(void);
		void EVENT_USB_Device_ControlRequest(void);
		void EVENT_USB_Device_StartOfFrame(void);

		bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
		                                         uint8_t* const ReportID,
		                                         const uint8_t ReportType,
		                                         void* ReportData,
		                                         uint16_t* const ReportSize);
		void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
		                                          const uint8_t ReportID,
		                                          const uint8_t ReportType,
		                                          const void* ReportData,
		                                          const uint16_t ReportSize);
#endif

