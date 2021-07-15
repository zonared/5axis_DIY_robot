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

//#use i2c(SLAVE, SDA=PIN_B1, SCL=PIN_B4, address=0x4C, FAST=100000, FORCE_HW, NO_STRETCH)
//#use i2c(SLAVE, SDA=PIN_B1, SCL=PIN_B4, address=0x4E, FAST=100000, FORCE_HW, NO_STRETCH)
#use i2c(SLAVE, SDA=PIN_B1, SCL=PIN_B4, address=0x50, FAST=100000, FORCE_HW, NO_STRETCH)	// Joint 3
//#use i2c(SLAVE, SDA=PIN_B1, SCL=PIN_B4, address=0x52, FAST=100000, FORCE_HW, NO_STRETCH)
//#use i2c(SLAVE, SDA=PIN_B1, SCL=PIN_B4, address=0x54, FAST=100000, FORCE_HW, NO_STRETCH)

#use RS232(Baud=9600,Xmit=PIN_B5,Rcv=PIN_B2,brgh1ok,ERRORS,Stream=COMMS)

#define hi(x)	(*(&x+1))
#define lo(x)	(*(&x))

#define RAND_MAX 255

#byte	TMR0 = 0x01
#bit	MathCarry = 0x03.0

//For Changing Edge Select H-L and L-H Directly
// instead of through ext_int_edge(l_to_h)
#bit EdgeDirection = 129.6

#byte RCSTA = 0x18
#byte	TXREG = 0x19
#byte RCREG = 0x1A
#byte TXIF = 12.4
#byte	RCIF = 12.5
#bit OERR = 24.1
#bit FERR = 24.2
#bit TrisB_RS232_TX = 134.5	// pin_B5 input/output
#bit TXEN = 152.5					// Transmit enable
#bit TRMT = 152.1					// Transmit empty

#byte	Option_Reg = 0x81

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

//#define AN1 battery voltage						// Pin 18
//#define AN0 steer pos								// Pin 17
#define MotorRST_Off		output_high(PIN_A2)	// Pin 1
#define MotorRST_On		output_low(PIN_A2)
#define Stepper_On		output_high(PIN_A3)	// Pin 2
#define Stepper_Off		output_low(PIN_A3)
#define CutFWD_On			output_high(PIN_A4)	// Pin 3
#define CutFWD_Off		output_low(PIN_A4)
#define spare				input(PIN_A5)			// Pin 4
#define SteerLeft_On		output_high(PIN_A6)	// Pin 15
#define SteerLeft_Off	output_low(PIN_A6)
#define SteerRight_On	output_high(PIN_A7)	// Pin 16
#define SteerRight_Off	output_low(PIN_A7)

// #define EnocderA Chn		input(PIN_B0)		// Pin 6
#define i2c_input_B1 	input(PIN_B1)			// Pin 7 i2c SDI
// #define B2 RS232 RX								// Pin 8
#define EncoderB			input(PIN_B3)			// Pin 9
#define i2c_input_B4		input(PIN_B4)			// Pin 10 i2c SCK
// #define B5 RS232 TX								// Pin 11
#define	Relay1_On	 	output_high(PIN_B6)	// Pin 12
#define	Relay1_Off	 	output_low(PIN_B6)
#define	Relay2_On	 	output_high(PIN_B7)	// Pin 13
#define	Relay2_Off	 	output_low(PIN_B7)

struct adc_control {
	short	ADON;
	short	unused;
	short	Go;
	int	Channel : 3;
	short	ADCS0;
	short ADCS1;
}		ADCON0;
#byte	ADCON0 = 0x1F

struct adc_control1 {
	int	unused : 3;
	int	VCFG : 2;
	short	ADCS2;
	short ADFM;
}		ADCON1;
#byte	ADCON1 = 0x9F

#byte	ADRESH = 0x1E
#byte	ADRESL = 0x9E
#byte	ANSEL = 0x9B

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
    }while (SSPCONbits.WCOL);           //do until write collision flag is clear
    if (SSPCONbits.SSPEN) SSPCONbits.CKP = 1;           //release the SCL line
}


#INT_SSP
void ssp_interrupt()
{
	//This isr routine uses one if block as opposed to multiple ones.
	//Using only one "if" avoids run time errors.  We handle only one interrupt at a time
	//all variables that are modified by an ISR must be declared as volatile
      
	clear_interrupt(INT_SSP);

	if(i2cStatus != 0x30)
	{
		i2cStatus = (SSPSTAT & 0b00101101);    //Mask out unimportant bits
		fprintf(COMMS,"int %u%c%c", i2cStatus, 11, 13);
                     // D/A, S, R/W, BF
      switch (i2cStatus){
      //State 1 RX address, this state is activated when a new I2c Call is received
      case i2c_STATE1: // STATE 1 master write last byte was ADDRESS
                       //SSPSTAT bits: D/A=0, S=1, R/W=0, BF=1
			value = SSPBUF;         //read buffer, clear BF
			RXBufferIndex = 0;          //clear index
			DAStatus=1; // next call is address inside memory
			if(SSPCONbits.SSPOV) SSPCONbits.SSPOV = 0;  //clear receive overflow indicator
			if (SSPCONbits.SSPEN) SSPCONbits.CKP = 1;      //release the SCL line
			fprintf(COMMS,"state1%c%c", 11, 13);
			break;
     
      case i2c_STATE2: // STATE2  last byte was data
                       //SSPSTAT bits D/A=1, S=1, R/W=0, BF=1
         value=SSPBUF; //read buffer, clear BF
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
			fprintf(COMMS,"state2%c%c", 11, 13);
         break;
         
      case i2c_STATE3:    // STATE 3  master read last byte was address
                          //SSPSTAT bits: D/A = 0, S=1, R/W=1, BF=0
         value = SSPBUF;         //dummy read
         if(RXBufferIndex>=RXBUFFER_SIZE)
         RXBufferIndex = 0;
         I2CWrite(RXBuffer[RXBufferIndex]);    //write back the index of status requested
         RXBufferIndex++;
			fprintf(COMMS,"state3%c%c", 11, 13);
         break;

      case i2c_STATE4: // STATE 4 last byte was data
                       //SSPSTAT bits: D/A=1, S=1, R/W=1, BF=0
         if(RXBufferIndex>=RXBUFFER_SIZE)
            RXBufferIndex = 0;
         I2CWrite(RXBuffer[RXBufferIndex]);    //write back the index of status requested
         RXBufferIndex++;
			fprintf(COMMS,"state4%c%c", 11, 13);
         break;

      case i2c_STATE5: // STATE 5 logic reset by NACK from master
          //SSPSTAT bits: D/A=1, S=1, R/W=0, BF=0, CKP=1
			fprintf(COMMS,"state5%c%c", 11, 13);
         break;
      default:
         if (SSPCONbits.SSPEN) SSPCONbits.CKP = 1;           //release the SCL line
			fprintf(COMMS,"state_x%c%c", 11, 13);
         break;
		}//end switch (i2cStatus)
	}// end if status !=30
}


/*
void ReadEEprom(void)
{
	iThisMotorNum = read_eeprom(10);
	iAnalogThreshold = read_eeprom(11);
	hi(lCurrPos) = read_eeprom(12);
	lo(lCurrPos) = read_eeprom(13);
	hi(lFullTravDist) = read_eeprom(14);
	lo(lFullTravDist) = read_eeprom(15);
	hi(lMtrStallPRE) = read_eeprom(16);
	lo(lMtrStallPRE) = read_eeprom(17);
}


void WriteEEprom(void)
{
	write_eeprom(10,iThisMotorNum);
	write_eeprom(11,iAnalogThreshold);
	write_eeprom(12,hi(lCurrPos));
	write_eeprom(13,lo(lCurrPos));
	write_eeprom(14,hi(lFullTravDist));
	write_eeprom(15,lo(lFullTravDist));
	write_eeprom(16,hi(lMtrStallPRE));
	write_eeprom(17,lo(lMtrStallPRE));
	SaveEEprom = OFF;
}
*/


//**********************************************
//Handles external interrupt
// toggles a bit and adds 1 to an encoder value
//
//**********************************************
#int_ext
Handle_ext()
{
	//EdgeDirection = !EdgeDirection;
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

	setup_timer_0(RTCC_INTERNAL|RTCC_DIV_128|RTCC_8_BIT);
	setup_timer_1(T1_INTERNAL|T1_DIV_BY_8);
	// Timer2 Registers Prescaler= 16 - TMR2 PostScaler = 5 - PR2 = 250 - Freq = 100.00 Hz - Period = 0.010000 seconds
	setup_timer_2(T2_DIV_BY_16,250,5);
	//CCP1CON = 0;
	//SSPcon = 0b000100010;

	setup_wdt(WDT_2304MS);			// Fire up the watch dog timer
	restart_wdt();                // Pat the watchdog timer

	//Option_Reg = 0b11010000;

	setup_adc(ADC_CLOCK_INTERNAL);
	setup_adc_ports(ADC_OFF);

	//setup_spi(SPI_MASTER | SPI_L_TO_H | SPI_CLK_DIV_64);

	//enable_interrupts(int_timer2);
	enable_interrupts(int_EXT);
	//enable_interrupts(INT_SSP);
	//enable_interrupts(int_RDA);
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

	// read the restart cause
	switch(restart_cause())
	{
		case WDT_FROM_SLEEP:
		{
			fprintf(COMMS,"Restart cause was Watchdog from sleep%c%c", 11, 13);
		}	break;

		case WDT_TIMEOUT:
		{
			fprintf(COMMS,"Restart cause was Watchdog timeout%c%c", 11, 13);
		}	break;

		case MCLR_FROM_SLEEP:
		{
			fprintf(COMMS,"Restart cause was master clear from sleep%c%c", 11, 13);
		}	break;

		case NORMAL_POWER_UP:
		{
			fprintf(COMMS,"Restart cause was normal powerup%c%c", 11, 13);
		}	break;

		default:
		{
			fprintf(COMMS,"Restart cause was other %u%c%c", restart_cause(), 11, 13);
		}	break;
	}

	for(;;)
	{
		restart_wdt();                // Pat the watchdog timer
		++iHeartbeat;

		delay_ms(500);
		fprintf(COMMS,"Encoder %05lu%c%c", lEncoder, 11, 13);
/*

		if(bDataRec)
		{
			fprintf(COMMS,"Buff=  0   1   2   3   4   5   6   7   8   9%c%c     ",11,13);
			for(i=0; i<10;++i)
			{
				fprintf(COMMS,"%03u ", inBuff[i]);
				inBuff[i]=0;
			}
			fprintf(COMMS,"%c%c",11,13);
			bDataRec = 0;
		}

		if(iWriteRec)
		{
			fprintf(COMMS,"Write received at %u%c%c", iWriteRec,11,13);
			iWriteRec = 0;
		}

		if(iDataWas)
		{
			fprintf(COMMS,"other data was %u%c%c", iDataWas,11,13);
			iWriteRec = 0;
		}
*/
	}
}
