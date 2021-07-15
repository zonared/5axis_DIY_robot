//
//	Version 1.0
//		Using 16F88
//
//

#include <16F88.h>

#fuses INTRC_IO,NOPROTECT,WDT,PUT,MCLR,BROWNOUT,NOLVP,NODEBUG,CCPB0

#device ADC=10	// Set ADC to 10bit
#device *=16

#use delay(clock=8000000, restart_wdt)

#use i2c(SLAVE, SDA=PIN_B1, SCL=PIN_B4, address=0x50, FAST=100000, FORCE_HW, NO_STRETCH)	// Joint 3

#use RS232(Baud=9600,Xmit=PIN_B5,Rcv=PIN_B2,brgh1ok,ERRORS,Stream=DEBUG)

#byte SSPBUF =  0x13
#byte SSPcon = 0x14
#byte CCP1CON = 0x17
#byte SSPstat = 0x94

#bit DA_BIT = SSPSTAT.5
#bit RW_BIT = SSPBUF.0

struct sspstat_status {
	short BF;
	short UA;
	short R_nW;
	short Start;
	short Stop;
	short D_nA;
	short CKE;
	short SMP;
} SSPSTATbits;
#byte SSPSTATbits = 0x94

struct sspcon_control {
	short SSPM_0;
	short SSPM_1;
	short SSPM_2;
	short SSPM_3;
	short CKP;
	short SSPEN;
	short SSPOV;
	short WCOL;
} SSPCONbits;
#byte SSPCONbits = 0x14

// #define EnocderA Chn		input(PIN_B0)		// Pin 6
#define i2c_input_B1 		input(PIN_B1)		// Pin 7 i2c SDI
// #define B2 RS232 RX					// Pin 8
#define EncoderB		input(PIN_B3)		// Pin 9
#define i2c_input_B4		input(PIN_B4)		// Pin 10 i2c SCK
// #define B5 RS232 TX					// Pin 11

#byte PORTB = 0x06

#define ON 1
#define OFF 0

#define i2c_STATE1      0b00001001 // 0x09 master write last was address
#define i2c_STATE2      0b00101001 // 0x29 master write last was data
#define i2c_STATE3      0b00001101 // 0x0d master read last was address
#define i2c_STATE4      0b00101100 // 0x2c master write last was data
#define i2c_STATE5      0b00101000 // 0x28
#define RXBUFFER_SIZE 14
// Reserve an array in ram for i2c buffer overlayed on working variables
#reserve 0x2A:0x37
int	RXBuffer[14];
#byte RXBuffer = 0x2A
int	iVersion = 15;			// iBuffer[0]
#byte iVersion = 0x2A
int	iCommand = 0;			// iBuffer[1]
#byte iCommand = 0x2B
int	iHeartbeat = 0;		// iBuffer[2]
#byte iHeartbeat = 0x2C
int	iCommsPacket = 0;		// iBuffer[3]
#byte iCommsPacket = 0x2D
long	lEncoder=8389;			// iBuffer[4]
#byte lEncoder = 0x2E


int	RXBufferIndex = 0;
int	iCommsPacketPre=0;
int	iCommsTimeout=0;
int	iUpdate=0;
int	iHeartbeatFast=0;
int	inBuff[50];
int	iWriteRec=0;
int	iDataWas=0;
short bDataRec=0;
static char DAStatus=0;
unsigned char i2cStatus, value;


void I2CWrite(unsigned char data){
    while(SSPSTATbits.BF);      //wait while buffer is full
    do{
        SSPCONbits.WCOL = 0;           //clear write collision flag
        SSPBUF = data;
    }
	while (SSPCONbits.WCOL);           //do until write collision flag is clear
		if (SSPCONbits.SSPEN) SSPCONbits.CKP = 1;           //release the SCL line
}


#INT_SSP
void ssp_interrupt()
{
	//This routine uses one if block as opposed to multiple ones.
	//Using only one "if" avoids run time errors.  We handle only one interrupt at a time
	//all variables that are modified by an ISR must be declared as volatile
      
	clear_interrupt(INT_SSP);

	if(i2cStatus != 0x30)
	{
		i2cStatus = (SSPSTAT & 0b00101101);    //Mask out unimportant bits
		fprintf(DEBUG,"int %u%c%c", i2cStatus, 11, 13);
                     // D/A, S, R/W, BF
		switch (i2cStatus){
		//State 1 RX address, this state is activated when a new I2c Call is received
		case i2c_STATE1:	// STATE 1 master write last byte was ADDRESS
							//SSPSTAT bits: D/A=0, S=1, R/W=0, BF=1
			value = SSPBUF;			//read buffer, clear BF
			RXBufferIndex = 0;		//clear index
			DAStatus=1; // next call is address inside memory
			if(SSPCONbits.SSPOV) SSPCONbits.SSPOV = 0;		//clear receive overflow indicator
			if (SSPCONbits.SSPEN) SSPCONbits.CKP = 1;		//release the SCL line
			fprintf(DEBUG,"state1%c%c", 11, 13);
			break;
     
		case i2c_STATE2:	// STATE2  last byte was data
							//SSPSTAT bits D/A=1, S=1, R/W=0, BF=1
			value=SSPBUF;		//read buffer, clear BF
			if(DAStatus==1)
			{
				RXBufferIndex=value;
				DAStatus=2;
			}
			else
			{
				RXBuffer[RXBufferIndex]=value;
				RXBufferIndex++;                    //increment index
				if (RXBufferIndex>=RXBUFFER_SIZE) RXBufferIndex = 0; //RZ
			}
			if (SSPCONbits.SSPEN) SSPCONbits.CKP = 1;           //release the SCL line
			fprintf(DEBUG,"state2%c%c", 11, 13);
			break;
         
		case i2c_STATE3:	// STATE 3  master read last byte was address
							//SSPSTAT bits: D/A = 0, S=1, R/W=1, BF=0
			value = SSPBUF;         //dummy read
			if(RXBufferIndex>=RXBUFFER_SIZE)
			RXBufferIndex = 0;
			I2CWrite(RXBuffer[RXBufferIndex]);    //write back the index of status requested
			RXBufferIndex++;
			fprintf(DEBUG,"state3%c%c", 11, 13);
			break;

		case i2c_STATE4:	// STATE 4 last byte was data
							//SSPSTAT bits: D/A=1, S=1, R/W=1, BF=0
			if(RXBufferIndex>=RXBUFFER_SIZE)
				RXBufferIndex = 0;
			I2CWrite(RXBuffer[RXBufferIndex]);    //write back the index of status requested
			RXBufferIndex++;
			fprintf(DEBUG,"state4%c%c", 11, 13);
			break;

		case i2c_STATE5:	// STATE 5 logic reset by NACK from master
							//SSPSTAT bits: D/A=1, S=1, R/W=0, BF=0, CKP=1
			fprintf(DEBUG,"state5%c%c", 11, 13);
			break;
				
		default:
			if (SSPCONbits.SSPEN) SSPCONbits.CKP = 1;           //release the SCL line
			fprintf(DEBUG,"state_x%c%c", 11, 13);
			break;
		}//end switch (i2cStatus)
	}// end if status !=30
}


//**********************************************
//Handles external interrupt
// Adds 1 to an encoder value
// or subtracts depending on B channel
//
//**********************************************
#int_ext
Handle_ext()
{
	if(EncoderB)
	{
		lEncoder++;
	}
	else
	{
		lEncoder--;
	}
}

void main()
{
	int i=0;
	disable_interrupts(INT_TIMER0);
	disable_interrupts(INT_TIMER1);
	disable_interrupts(INT_TIMER2);
	disable_interrupts(INT_CCP1);
	disable_interrupts(INT_EXT);
	disable_interrupts(int_AD);
	disable_interrupts(int_rda);
	disable_interrupts(int_TBE);
	disable_interrupts(INT_SSP);
	disable_interrupts(global);

	set_tris_a(0b00100011);
	set_tris_b(0b00111111);
	port_b_pullups(TRUE);

	setup_wdt(WDT_2304MS);			// Fire up the watch dog timer
	restart_wdt();                // Pat the watchdog timer

	setup_adc(ADC_CLOCK_INTERNAL);
	setup_adc_ports(ADC_OFF);

	enable_interrupts(int_EXT);
	enable_interrupts(global);

	// Disable i2c
	SSPcon = 0b00010110;
	set_tris_b(0b00111101);
	output_high(PIN_B1);		// Pin 7 i2c SDI

	// Enable i2c
	SSPcon = 0b00110110;
	set_tris_b(0b00111111);
	// Enable i2c interrupts after centering the steering
	enable_interrupts(INT_SSP);

	for(;;)
	{
		restart_wdt();                // Pat the watchdog timer
		++iHeartbeat;

		delay_ms(500);
		fprintf(DEBUG,"Encoder %05lu%c%c", lEncoder, 11, 13);
	}
}
