//##########################################################
//##                      R O B O T I S                   ##
//## CM-700 (Atmega2561) Example code for Dynamixel.      ##
//##                                           2009.11.10 ##
//##########################################################

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>

#include "serial.h"
#include "dynamixel.h"

#define PI	3.141592f


/// Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_GOAL_SPEED_L		32
#define P_GOAL_SPEED_H		33

// Defulat setting
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define NUM_ACTUATOR		6 // Number of actuator
#define STEP_THETA			(PI / 100.0f) // Large value is more fast
#define CONTROL_PERIOD		(10) // msec (Large value is more slow) 

void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);

int main(void)
{
	int id[NUM_ACTUATOR];
	float phase[NUM_ACTUATOR];
	//int AmpPos = 2048; // for EX series
	int i;

	serial_initialize(57600);
	dxl_initialize( 0, DEFAULT_BAUDNUM ); // Not using device index
	sei();	// Interrupt Enable	
	
	printf( "\n\nSyncWrite example for CM-700\n\n" );
		
	for( i=0; i<NUM_ACTUATOR; i++ )
	{
		id[i] = i+1;
		phase[i] = 2*PI * (float)i / (float)NUM_ACTUATOR;
	}
	
	// Set goal speed
	dxl_write_word( BROADCAST_ID, P_GOAL_SPEED_L, 1000);

	return 0;
}

// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

	case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

	case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

	case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;

	case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

	case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

	default:
		printf("This is unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		printf("Instruction code error!\n");
}

