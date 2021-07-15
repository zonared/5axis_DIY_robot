// This #include statement was automatically added by the Particle IDE.
#include <blynk.h>
#define BLYNK_PRINT Serial  // Set serial output for debug prints
//#define BLYNK_DEBUG       // Uncomment this to see detailed prints

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

#define i2c_SDA D0
#define i2c_SCL D1

#define JT1_PWM A4
#define JT1_Dir A1
#define JT1_Enc D5

#define JT2_PWM D2
#define JT2_Dir A0
#define JT2_Enc D6

#define JT3_PWM RX
#define JT3_Dir A2
#define JT3_Enc D7

#define JT4_PWM TX
#define JT4_Dir A3
#define JT4_Enc D3

#define JT5_PWM WKP
#define JT5_Dir A5
#define JT5_Enc D4

uint16_t iTargetRange=0;
unsigned long lastCounterUpdate = 0;
unsigned long lastCounter500ms = 0;
int iCommsPacketPre=0;
int iCommsTimeout=0;

uint8_t iCalSpeed=100;
uint8_t iCalJoint=1;
uint16_t lCalPosPre;
uint8_t iJogSpeed=100;
unsigned long lastDelay = 0;
int16_t Target_Change[6];
int8_t Speed_Change[6];
volatile int16_t Joint_pos[6];
int16_t PWM_Freq=500;
uint8_t crc = 0x83;
uint8_t iCRC_calc=0;
volatile uint32_t debounce[6];
    
int iCases=0;
int iPause=0;
int iJointCycle=1;
//int iJointPos=1;
int iBufferPtr=0;
int iRequestType=0;
int iRequestJoint=0;
bool bMotorStalled;
bool bStringFound;
bool bSerialStayAwake=0;
bool bSaveEEPROM=FALSE;
bool bJogging=FALSE;
bool bROS_Control=FALSE;
bool bROS_Control_Vel=FALSE;
uint16_t iEEPROM_Ptr=1;
uint8_t iRec_Ptr=0;
uint8_t iRx_Buf[25];
uint8_t iTx_Buf[25];
char cBuffer[20];
uint8_t iCommsPtr=0;
String sBuffer = "Hello String";
volatile bool toggle;
uint8_t i2c_buf[10];

bool bDataRec1 = FALSE;
bool bDataRec2 = FALSE;
bool bDataRec3 = FALSE;
bool bDataRec4 = FALSE;

Timer MotorStalled(1000, motorStall, FALSE);     // timer to detect motor is stalled

typedef struct motor_dut{
    bool bForward;
    bool bCalibrate=FALSE;
    bool bRunning=FALSE;
    uint16_t dVelocity = 0;
    uint16_t dVel_pre = 0;
    uint16_t dMax_Trav = 0;
    uint16_t dPosition = 0;
    int pwm_pin;
    int dir_pin;
    int enc_pin;
};

typedef struct data_out{
    uint8_t 	Heartbeat;   // iBuffer[0] 0x00
    uint8_t 	CommsPacket; // iBuffer[1] 0x01
    uint16_t	JointPos[6]; // iBuffer[2] 0x02
};

typedef struct data_in{
    uint16_t	Target[6];   // iBuffer[00] 0x00 - 0x0b
    uint8_t	    Speed[6];    // iBuffer[12] 0x0c - 0x11
};

typedef union data_union_in{
    data_in DATA;
    uint8_t REG[18];
};

data_union_in comms_in;

typedef union data_union_out{
    data_out DATA;
    uint8_t REG[14];
};

data_union_out comms_out;

motor_dut   JointMotors[6];

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "?????";

/// telnet defaults to port 23
TCPServer server = TCPServer(23);
TCPClient client;


//**********************************************
// Handles external interrupt for JT1 encoder
//  adds or subtracts 1 to/from an encoder value
//  depends on motor direction
//**********************************************
void JT1_Enc_Count()
{
//    if(micros() - debounce[1] < 1000) return; // ignore repeated trigger with 1ms
//    debounce[1] = micros();

	if(JointMotors[1].bForward)
	{
		++Joint_pos[1];
	}
	else
	{
		if(Joint_pos[1] > 0)
    		{
			--Joint_pos[1];
		}
	}
}


//**********************************************
// Handles external interrupt for JT2 encoder
//  adds or subtracts 1 to/from an encoder value
//  depends on motor direction
//**********************************************
void JT2_Enc_Count()
{
//    if(micros() - debounce[2] < 1000) return; // ignore repeated trigger with 1ms
//    debounce[2] = micros();

	if(JointMotors[2].bForward)
	{
	    ++Joint_pos[2];
	}
	else
	{
		if(Joint_pos[2] > 0)
		{
			--Joint_pos[2];
		}
	}
}


//**********************************************
// Handles external interrupt for JT3 encoder
//  adds or subtracts 1 to/from an encoder value
//  depends on motor direction
//**********************************************
void JT3_Enc_Count()
{
//    if(micros() - debounce[3] < 1000) return; // ignore repeated trigger with 1ms
//    debounce[3] = micros();

	if(JointMotors[3].bForward)
	{
		++Joint_pos[3];
	}
	else
	{
		if(Joint_pos[3] > 0)
		{
			--Joint_pos[3];
		}
	}
}


//**********************************************
// Handles external interrupt for JT4 encoder
//  adds or subtracts 1 to/from an encoder value
//  depends on motor direction
//**********************************************
void JT4_Enc_Count()
{
//    if(micros() - debounce[4] < 1000) return; // ignore repeated trigger with 1ms
//    debounce[4] = micros();

	if(JointMotors[4].bForward)
	{
		++Joint_pos[4];
	}
	else
	{
		if(Joint_pos[4] > 0)
    		{
			--Joint_pos[4];
		}
	}
}


//**********************************************
// Handles external interrupt for JT5 encoder
//  adds or subtracts 1 to/from an encoder value
//  depends on motor direction
//**********************************************
void JT5_Enc_Count()
{
    if(micros() - debounce[5] < 20000) return; // ignore repeated trigger with 1ms
    debounce[5] = micros();

	if(JointMotors[5].bForward)
	{
		++Joint_pos[5];
	}
	else
	{
		if(Joint_pos[5] > 0)
		{
			--Joint_pos[5];
		}
	}
}


//**********************************************
// Function triggered via a timer interrupt
//
//**********************************************
void motorStall()
{
	bMotorStalled = TRUE;
}


//**********************************************
// Function to read i2c encoder valve
//  
//  
//**********************************************
uint16_t readi2c_encoder(uint8_t joint)
{
    if(client.connected()) server.printlnf("Encoder set reg :%u",millis());
    uint16_t return_val;
    // Read enocder via i2c bus
    // Joint_1 address is 0x4c or 0x26 right shifted 1
    // Joint_2 address is 0x4e or 0x27 right shifted 1
    // Joint_3 address is 0x50 or 0x28 right shifted 1
    // Joint_4 address is 0x52 or 0x29 right shifted 1
    // Joint_5 address is 0x54 or 0x2a right shifted 1
    digitalWrite(DAC, TRUE);
    Wire.beginTransmission(0x26 + joint - 1); // transmit to slave device
    Wire.write(0);             // sends one byte
    Wire.endTransmission();    // stop transmitting
    //Wire.reset();

    if(client.connected()) server.printlnf("About to read encoder :%u",millis());

    if(Wire.requestFrom(0x26 + joint - 1, 7) > 0)    // request bytes from slave device
    {
        uint8_t i2c_ptr=0;
        while(Wire.available())   // slave may send less than requested
		{
			i2c_buf[i2c_ptr] = Wire.read();    // receive a byte as character
			++i2c_ptr;
		}
		return_val = ((uint16_t)i2c_buf[5]<<8) + i2c_buf[4];
	}
    else
    {
/*        int clock_lost=0;
        pinMode(i2c_SCL, INPUT);
        while(!digitalRead(i2c_SCL) && clock_lost < 10)
        {
            pinMode(i2c_SCL, OUTPUT);
            digitalWrite(i2c_SCL, TRUE);
            delay(1);
            digitalWrite(i2c_SCL, FALSE);
            delay(1);
            pinMode(i2c_SCL, INPUT);
            ++clock_lost;
        }*/
        //Serial.println("i2c slave stuck");
        // if we didnt get any data, then just give the old data back
        return_val = comms_out.DATA.JointPos[joint];
    }
    //Wire.reset();
    digitalWrite(DAC, FALSE);
    if(client.connected()) server.printlnf("Reading finished :%u",millis());
    return return_val;
}



//**********************************************
// Function to read i2c encoder valve
//  
//  
//**********************************************
void reseti2c_encoder(uint8_t joint, uint16_t value)
{
    uint8_t byte_to_write=0;
    Wire.beginTransmission(0x26 + joint - 1); // transmit to slave device
    Wire.write(4);              // write the register to change
    byte_to_write = uint8_t(0x00ff & value);
    Wire.write(byte_to_write);   // write first byte
    Wire.endTransmission();     // stop transmitting

    delay(25);
    Wire.reset();
    
    Wire.beginTransmission(0x26 + joint - 1); // transmit to slave device
    Wire.write(5);              // write the register to change
    byte_to_write = uint8_t(0x00ff & (value >> 8));
    Wire.write(byte_to_write);// write second byte
    Wire.endTransmission();     // stop transmitting
}


void setup() {
    bool bEEPROM_Blank=FALSE;

    pinMode(DAC, OUTPUT);
    pinMode(i2c_SDA, INPUT);
    pinMode(i2c_SCL, INPUT);

    pinMode(JT1_PWM, OUTPUT);
    pinMode(JT1_Dir, OUTPUT);
    pinMode(JT1_Enc, INPUT_PULLUP);
    JointMotors[1].pwm_pin = JT1_PWM;
    JointMotors[1].dir_pin = JT1_Dir;
    JointMotors[1].enc_pin = JT1_Enc;

    pinMode(JT2_PWM, OUTPUT);
    pinMode(JT2_Dir, OUTPUT);
    pinMode(JT2_Enc, INPUT_PULLUP);
    JointMotors[2].pwm_pin = JT2_PWM;
    JointMotors[2].dir_pin = JT2_Dir;
    JointMotors[2].enc_pin = JT2_Enc;

    pinMode(JT3_PWM, OUTPUT);
    pinMode(JT3_Dir, OUTPUT);
    pinMode(JT3_Enc, INPUT_PULLUP);
    JointMotors[3].pwm_pin = JT3_PWM;
    JointMotors[3].dir_pin = JT3_Dir;
    JointMotors[3].enc_pin = JT3_Enc;

    pinMode(JT4_PWM, OUTPUT);
    pinMode(JT4_Dir, OUTPUT);
    pinMode(JT4_Enc, INPUT_PULLUP);
    JointMotors[4].pwm_pin = JT4_PWM;
    JointMotors[4].dir_pin = JT4_Dir;
    JointMotors[4].enc_pin = JT4_Enc;

    pinMode(JT5_PWM, OUTPUT);
    pinMode(JT5_Dir, OUTPUT);
    pinMode(JT5_Enc, INPUT_PULLUP);
    JointMotors[5].pwm_pin = JT5_PWM;
    JointMotors[5].dir_pin = JT5_Dir;
    JointMotors[5].enc_pin = JT5_Enc;
    
    Serial.begin(115200);
    Wire.setSpeed(400000);
    Wire.begin(); // start i2c bus

    // start listening for clients
    server.begin();
    
    // Connect the particle cloud
    Particle.connect();
    
    while(!Particle.connected())
    {
	    Particle.process();
    }
    while(Serial.available()) int dump=Serial.read();
/*    while(!bStringFound)
    {
        if(Serial.available())
        {
            if(Serial.read() == 13) bStringFound = TRUE;
        }
    }
*/
    Serial.printlnf("Running");
    Serial.printlnf("JointMotor struct size:%u", sizeof(JointMotors));
    
    // Read eepROM element zero to check if its factory default ie 0xFF
    uint8_t eeprom_version = EEPROM.read(0);
    if(eeprom_version == 0xFF){
        Serial.printlnf("EEPROM is blank");
        bEEPROM_Blank=TRUE;
    }

    // Setup encoder external interrupts
    // No longer used due to issue with false interrupt triggers
    // now using external dedicated micros for encoders and communicated via i2c
//    Serial.printlnf("JT1 Encoder ISR %u", attachInterrupt(JT1_Enc, JT1_Enc_Count, CHANGE, 1));

    // Setup encoder external interrupts
//    Serial.printlnf("JT2 Encoder ISR %u", attachInterrupt(JT2_Enc, JT2_Enc_Count, CHANGE, 1));

    // Setup encoder external interrupts
//    Serial.printlnf("JT3 Encoder ISR %u", attachInterrupt(JT3_Enc, JT3_Enc_Count, CHANGE, 1));

    // Setup encoder external interrupts
//    Serial.printlnf("JT4 Encoder ISR %u", attachInterrupt(JT4_Enc, JT4_Enc_Count, CHANGE, 1));

    // Setup encoder external interrupts
    Serial.printlnf("JT5 Encoder ISR %u", attachInterrupt(JT5_Enc, JT5_Enc_Count, CHANGE, 1));

    if(bEEPROM_Blank){
        EEPROM.write(0,1); // Version 0.1
        iEEPROM_Ptr = 1;
        EEPROM.put(iEEPROM_Ptr, JointMotors[1]);
        iEEPROM_Ptr += sizeof(JointMotors) + 5;
        EEPROM.put(iEEPROM_Ptr, JointMotors[2]);
        iEEPROM_Ptr += sizeof(JointMotors) + 5;
        EEPROM.put(iEEPROM_Ptr, JointMotors[3]);
        iEEPROM_Ptr += sizeof(JointMotors) + 5;
        EEPROM.put(iEEPROM_Ptr, JointMotors[4]);
        iEEPROM_Ptr += sizeof(JointMotors) + 5;
        EEPROM.put(iEEPROM_Ptr, JointMotors[5]);
    }
    else{
        iEEPROM_Ptr = 1;
        EEPROM.get(iEEPROM_Ptr, JointMotors[1]);
        comms_out.DATA.JointPos[1] = JointMotors[1].dPosition;
        Joint_pos[1] = comms_out.DATA.JointPos[1];
        Serial.printlnf("Read JT1 Pos:%u from EEPROM:%u",JointMotors[1].dPosition, iEEPROM_Ptr);
        
        iEEPROM_Ptr = iEEPROM_Ptr + (sizeof(JointMotors) + 5);
        EEPROM.get(iEEPROM_Ptr, JointMotors[2]);
        comms_out.DATA.JointPos[2] = JointMotors[2].dPosition;
        Joint_pos[2] = comms_out.DATA.JointPos[2];
        Serial.printlnf("Read JT2 Pos:%u from EEPROM:%u",JointMotors[2].dPosition, iEEPROM_Ptr);
        
        iEEPROM_Ptr = iEEPROM_Ptr + (sizeof(JointMotors) + 5);
        EEPROM.get(iEEPROM_Ptr, JointMotors[3]);
        comms_out.DATA.JointPos[3] = JointMotors[3].dPosition;
        Joint_pos[3] = comms_out.DATA.JointPos[3];
        Serial.printlnf("Read JT3 Pos:%u from EEPROM:%u",JointMotors[3].dPosition, iEEPROM_Ptr);
        
        iEEPROM_Ptr = iEEPROM_Ptr + (sizeof(JointMotors) + 5);
        EEPROM.get(iEEPROM_Ptr, JointMotors[4]);
        comms_out.DATA.JointPos[4] = JointMotors[4].dPosition;
        Joint_pos[4] = comms_out.DATA.JointPos[4];
        Serial.printlnf("Read JT4 Pos:%u from EEPROM:%u",JointMotors[4].dPosition, iEEPROM_Ptr);
        
        iEEPROM_Ptr = iEEPROM_Ptr + (sizeof(JointMotors) + 5);
        EEPROM.get(iEEPROM_Ptr, JointMotors[5]);
        comms_out.DATA.JointPos[5] = JointMotors[5].dPosition;
        Joint_pos[5] = comms_out.DATA.JointPos[5];
        Serial.printlnf("Read JT5 Pos:%u from EEPROM:%u",JointMotors[5].dPosition, iEEPROM_Ptr);
    }

    digitalWrite(JointMotors[1].dir_pin, TRUE);
    analogWrite(JointMotors[1].pwm_pin,comms_in.DATA.Speed[1]);

    digitalWrite(JointMotors[2].dir_pin, TRUE);
    analogWrite(JointMotors[2].pwm_pin,comms_in.DATA.Speed[2]);

    digitalWrite(JointMotors[3].dir_pin, TRUE);
    analogWrite(JointMotors[3].pwm_pin,comms_in.DATA.Speed[3]);

    digitalWrite(JointMotors[4].dir_pin, TRUE);
    analogWrite(JointMotors[4].pwm_pin,comms_in.DATA.Speed[4]);

    digitalWrite(JointMotors[5].dir_pin, TRUE);
    analogWrite(JointMotors[5].pwm_pin,comms_in.DATA.Speed[5]);
    
    Blynk.begin(auth);
    // Wait until we have received comms, or if we received a serial char
	while(!bROS_Control && !bSerialStayAwake)
	{
	    Particle.process();
        Blynk.run();
        HandleSerial();
        
        Blynk.virtualWrite(V11,comms_out.DATA.JointPos[1]);
        Blynk.virtualWrite(V12,comms_out.DATA.JointPos[2]);
        Blynk.virtualWrite(V13,comms_out.DATA.JointPos[3]);
        Blynk.virtualWrite(V14,comms_out.DATA.JointPos[4]);
        Blynk.virtualWrite(V15,comms_out.DATA.JointPos[5]);

        Blynk.virtualWrite(V16,JointMotors[1].dMax_Trav);
        Blynk.virtualWrite(V17,JointMotors[2].dMax_Trav);
        Blynk.virtualWrite(V18,JointMotors[3].dMax_Trav);
        Blynk.virtualWrite(V19,JointMotors[4].dMax_Trav);
        Blynk.virtualWrite(V20,JointMotors[5].dMax_Trav);

        comms_out.DATA.JointPos[1] = readi2c_encoder(1);//Joint_pos[1];
        comms_out.DATA.JointPos[2] = readi2c_encoder(2);//Joint_pos[2];
        comms_out.DATA.JointPos[3] = readi2c_encoder(3);//Joint_pos[3];
        comms_out.DATA.JointPos[4] = readi2c_encoder(4);//Joint_pos[4];
        comms_out.DATA.JointPos[5] = Joint_pos[5];

        if(bSaveEEPROM){
            bSaveEEPROM = FALSE;
            HandleSave();
        }

/*        Wire.beginTransmission(0x28); // transmit to slave device #0x50
        Wire.write(0);             // sends one byte
        Wire.endTransmission();    // stop transmitting

        Wire.requestFrom(0x28, 6);    // request 6 bytes from slave device 0x50

        uint8_t i2c_buf[10];
        uint8_t i2c_ptr=0;
        while(Wire.available()){   // slave may send less than requested
            i2c_buf[i2c_ptr] = Wire.read();    // receive a byte as character
            Serial.printf("%03u ",i2c_buf[i2c_ptr]);
            ++i2c_ptr;
        }
        Serial.printlnf(" ");
        Blynk.virtualWrite(V11,((uint16_t)i2c_buf[5]<<8) + i2c_buf[4]);
*/
        if (client.connected()) {
        // echo all available bytes back to the client
        while (client.available()) {
          server.write(client.read());
        }
        } else {
        // if no client is yet connected, check for a new connection
        client = server.available();
        }

	}
    iCommsPacketPre = comms_out.DATA.CommsPacket;
    // if this was woken via serial comms, then stay awake
    if(bSerialStayAwake){
        // Connect the particle cloud
        Serial.printlnf("Manual Control");
    }
    else{
        // Disconnect the particle cloud so we can concentrate all our efforts on robot arm
        Blynk.disconnect();
        while(Blynk.connected())
        {
            Particle.process();
            Blynk.run();
        }
        Particle.disconnect();
        while(Particle.connected())
        {
            Particle.process();
        }
        WiFi.off();
        Serial.printlnf("Control via ROS");
    }
//    bROS_Control = TRUE;
}


void loop() {

	if (millis() - lastCounterUpdate >= 10) {
		lastCounterUpdate = millis();

        // measure pulses per 100ms for PID
//        JointMotors[iJointCycle].dVelocity = comms_out.DATA.JointPos[iJointCycle] - JointMotors[iJointCycle].dVel_pre;
//        JointMotors[iJointCycle].dVel_pre = comms_out.DATA.JointPos[iJointCycle];

        //Serial.printlnf("JT4 Encoder:%u - Vel:%u - speed:%d", i2c_out.DATA.JT4_Pos, dJT4_Velocity, i2c_in.DATA.JT4_Speed);

        if((iCommsPacketPre == comms_out.DATA.CommsPacket) && !bSerialStayAwake)
        {
            ++iCommsTimeout;
        }
        if(iCommsTimeout > 200)
        {
            HandleSave();
            Blynk.disconnect();
        	delay(1000);
            System.sleep(SLEEP_MODE_DEEP,1);
        }
        
        ++comms_out.DATA.Heartbeat;

        if(!bROS_Control)
        {
            comms_out.DATA.JointPos[1] = readi2c_encoder(1);//Joint_pos[1];
            comms_out.DATA.JointPos[2] = readi2c_encoder(2);//Joint_pos[2];
            comms_out.DATA.JointPos[3] = readi2c_encoder(3);//Joint_pos[3];
            comms_out.DATA.JointPos[4] = readi2c_encoder(4);//Joint_pos[4];
            comms_out.DATA.JointPos[5] = Joint_pos[5];
    
            Blynk.virtualWrite(V11,comms_out.DATA.JointPos[1]);
            Blynk.virtualWrite(V12,comms_out.DATA.JointPos[2]);
            Blynk.virtualWrite(V13,comms_out.DATA.JointPos[3]);
            Blynk.virtualWrite(V14,comms_out.DATA.JointPos[4]);
            Blynk.virtualWrite(V15,comms_out.DATA.JointPos[5]);
    
            Blynk.virtualWrite(V16,JointMotors[1].dMax_Trav);
            Blynk.virtualWrite(V17,JointMotors[2].dMax_Trav);
            Blynk.virtualWrite(V18,JointMotors[3].dMax_Trav);
            Blynk.virtualWrite(V19,JointMotors[4].dMax_Trav);
            Blynk.virtualWrite(V20,JointMotors[5].dMax_Trav);
    
            Blynk.virtualWrite(V36,comms_in.DATA.Speed[1]);
            Blynk.virtualWrite(V37,comms_in.DATA.Speed[2]);
            Blynk.virtualWrite(V38,comms_in.DATA.Speed[3]);
            Blynk.virtualWrite(V39,comms_in.DATA.Speed[4]);
            Blynk.virtualWrite(V40,comms_in.DATA.Speed[5]);
        }
//        Serial.printlnf("JT1=%04u JT2=%04u JT3=%04u JT4=%04u JT5=%04u", dumb_counter[1],dumb_counter[2],dumb_counter[3],dumb_counter[4],dumb_counter[5]);

    }

    if(iCommsPacketPre != comms_out.DATA.CommsPacket)
    {
        iCommsPacketPre = comms_out.DATA.CommsPacket;
        iCommsTimeout = 0;
    }

/*    if(client.connected()) {
        if(millis() - lastCounter500ms >= 500) {
            lastCounter500ms  = millis();

            server.printlnf("Comm packet %u",comms_out.DATA.CommsPacket);
            server.printlnf("JT0     JT1     JT2     JT3     JT4     JT5     SP0 SP1 SP2 SP3 SP4 SP5");
            // Transfer new targets and speed into REG
            iCommsPtr = 0;
            while(iCommsPtr < 18)
            {
                server.printf("%03u ",comms_in.REG[iCommsPtr]);
                ++iCommsPtr;
            }
            server.println();
        }
    }
    else
    {
        // if no client is yet connected, check for a new connection
        client = server.available();
    }
*/

    if(!client.connected())
    {
        // if no client is yet connected, check for a new connection
        client = server.available();
    }


    if(bROS_Control)
    {
        if(Serial.available())
        {
            if(!bDataRec1)
            {
                if(client.connected()) server.printlnf("Data received at :%u",millis());
                bDataRec1 = TRUE;
            }
            iRx_Buf[iRec_Ptr] = Serial.read();

            ++iRec_Ptr;
            if(iRec_Ptr > 18)
            {
                crc = 0x83; // seed the crc
            	for(iCRC_calc=0; iCRC_calc<18;iCRC_calc++)
            	{
            	    // XOR all bytes and store in crc
            	    crc ^= iRx_Buf[iCRC_calc];
            	}

                // Check the packet's CRC byte against calc'd crc
                if(crc == iRx_Buf[18])
                {
                    if(client.connected()) server.printlnf("Packet received at :%u",millis());
                    bDataRec1 = FALSE;
                    bDataRec2 = FALSE;

                    // Success, data is valid, now transfer data to and from our registers
                    iRec_Ptr = 0;
                    ++comms_out.DATA.CommsPacket;

                    // Using a TCP client we can monitor data received, but must be connected first
                    if(client.connected()) server.printlnf("Comm packet %u, vel mode:%u",comms_out.DATA.CommsPacket,bROS_Control_Vel);
                    if(client.connected()) server.printlnf("JT0     JT1     JT2     JT3     JT4     JT5     SP0 SP1 SP2 SP3 SP4 SP5");

                    // Transfer new targets and speed into REG
                    iCommsPtr = 0;
                    while(iCommsPtr < 18)
                    {
                        comms_in.REG[iCommsPtr] = iRx_Buf[iCommsPtr];
                        if(client.connected()) server.printf("%03u ",comms_in.REG[iCommsPtr]);
                        ++iCommsPtr;
                    }
                    if(client.connected()) server.println();

                    // Now update all joints
                    // rate govened by the ROS node calling this photon (currently 10Hz)
                    for(iJointCycle=1; iJointCycle<=5; iJointCycle++)
                    {
                        // Joints 1 to 4 might be different control type
                        if(iJointCycle < 5)
                        {
                            if(bROS_Control_Vel)
                            {
                                // This is velocity mode
                                HandleJointsVel(iJointCycle);
                            }
                            else
                            {
                                // If not velocity then is position controlled
                                HandleJoints(iJointCycle);
                            }
                        }
                        else
                        {
                            // Joint 5 is a position controlled joint
                            HandleJoints(iJointCycle);
                        }
                        delay(1); // might have to delay between i2c handling, maybe
                    }
                    // Cycle to next joint, one per data packet
                    //++iJointCycle;
                    //if(iJointCycle>5) iJointCycle=1;

                    // Send current position back to ROS
                    iCommsPtr = 0;
                    while(iCommsPtr < 14)
                    {
                        // Prepair position data for sending to ROS
                        iTx_Buf[iCommsPtr] = comms_out.REG[iCommsPtr];
                        ++iCommsPtr;
                    }

                    // Build CRC for transmission packet
                    crc = 0x83;
                	for(iCRC_calc=0; iCRC_calc<14;iCRC_calc++)
                	{
                		crc ^= iTx_Buf[iCRC_calc];
                	}
                    iTx_Buf[14] = crc; // attached crc
                    
                    if(client.connected()) server.printlnf("Packet sending at :%u",millis());

                    // Send data back to ROS
                    iCommsPtr = 0;
                    while(iCommsPtr < 15)
                    {
                        Serial.write(iTx_Buf[iCommsPtr]);
                        ++iCommsPtr;
                    }
                    
                    if(client.connected()) server.printlnf("Packet sent at :%u",millis());
                }
                else
                {
                    if(!bDataRec2)
                    {
                        if(client.connected()) server.printlnf("Packet failed at :%u",millis());
                        bDataRec2  = TRUE;
                    }

                    // crc check failed, remove one char from the start of the buffer
                    iCommsPtr = 0;
                    while(iCommsPtr < sizeof(iRx_Buf)-1)
                    {
                        // Shift the rec buffer to the left by one byte
                        iRx_Buf[iCommsPtr] = iRx_Buf[iCommsPtr + 1];
                        ++iCommsPtr;
                    }
                    --iRec_Ptr; // decrement rec buffer pointer
                }
            }
        }
    }
    else
    {
        Particle.process();
        HandleSerial();
        Blynk.run();
    }

    if(bSaveEEPROM){
        bSaveEEPROM = FALSE;
        HandleSave();
    }

}


//**********************************************
// Function to handle Blynk jog button press
//  for JT1 Forward
//  
//**********************************************
BLYNK_WRITE(V0)
{
    if (param.asInt()) {
        Serial.printlnf("JT1 forward ON");
        JointMotors[1].bForward = TRUE;
        digitalWrite(JointMotors[1].dir_pin, JointMotors[1].bForward);
        analogWrite(JointMotors[1].pwm_pin,iJogSpeed,PWM_Freq);
    } else {
        analogWrite(JointMotors[1].pwm_pin,0);
        Serial.printlnf("JT1 forward OFF");
    }
}


//**********************************************
// Function to handle Blynk jog button press
//  for JT1 Reverse
//  
//**********************************************
BLYNK_WRITE(V1)
{
    if (param.asInt()) {
        Serial.printlnf("JT1 reverse ON");
        JointMotors[1].bForward = FALSE;
        digitalWrite(JointMotors[1].dir_pin, JointMotors[1].bForward);
        analogWrite(JointMotors[1].pwm_pin,iJogSpeed,PWM_Freq);
    } else {
        analogWrite(JointMotors[1].pwm_pin,0);
        Serial.printlnf("JT1 reverse OFF");
    }
}


//**********************************************
// Function to handle Blynk jog button press
//  for JT2 Forward
//  
//**********************************************
BLYNK_WRITE(V2)
{
    if (param.asInt()) {
        Serial.printlnf("JT2 forward ON");
        JointMotors[2].bForward = TRUE;
        digitalWrite(JointMotors[2].dir_pin, JointMotors[2].bForward);
        analogWrite(JointMotors[2].pwm_pin,iJogSpeed,PWM_Freq);
    } else {
        analogWrite(JointMotors[2].pwm_pin,0);
        Serial.printlnf("JT2 forward OFF");
    }
}


//**********************************************
// Function to handle Blynk jog button press
//  for JT2 Reverse
//  
//**********************************************
BLYNK_WRITE(V3)
{
    if (param.asInt()) {
        Serial.printlnf("JT2 reverse ON");
        JointMotors[2].bForward = FALSE;
        digitalWrite(JointMotors[2].dir_pin, JointMotors[2].bForward);
        analogWrite(JointMotors[2].pwm_pin,iJogSpeed,PWM_Freq);
    } else {
        analogWrite(JointMotors[2].pwm_pin,0);
        Serial.printlnf("JT2 reverse OFF");
    }
}


//**********************************************
// Function to handle Blynk jog button press
//  for JT3 Forward
//  
//**********************************************
BLYNK_WRITE(V4)
{
    if (param.asInt()) {
        Serial.printlnf("JT3 forward ON");
        JointMotors[3].bForward = TRUE;
        digitalWrite(JointMotors[3].dir_pin, JointMotors[3].bForward);
        analogWrite(JointMotors[3].pwm_pin,iJogSpeed,PWM_Freq);
    } else {
        analogWrite(JointMotors[3].pwm_pin,0);
        Serial.printlnf("JT3 forward OFF");
    }
}


//**********************************************
// Function to handle Blynk jog button press
//  for JT3 Reverse
//  
//**********************************************
BLYNK_WRITE(V5)
{
    if (param.asInt()) {
        Serial.printlnf("JT3 reverse ON");
        JointMotors[3].bForward = FALSE;
        digitalWrite(JointMotors[3].dir_pin, JointMotors[3].bForward);
        analogWrite(JointMotors[3].pwm_pin,iJogSpeed,PWM_Freq);
    } else {
        analogWrite(JointMotors[3].pwm_pin,0);
        Serial.printlnf("JT3 reverse OFF");
    }
}


//**********************************************
// Function to handle Blynk jog button press
//  for JT4 Forward
//  
//**********************************************
BLYNK_WRITE(V6)
{
    if (param.asInt()) {
        Serial.printlnf("JT4 forward ON");
        JointMotors[4].bForward = TRUE;
        digitalWrite(JointMotors[4].dir_pin, JointMotors[4].bForward);
        analogWrite(JointMotors[4].pwm_pin,iJogSpeed,PWM_Freq);
    } else {
        analogWrite(JointMotors[4].pwm_pin,0);
        Serial.printlnf("JT4 forward OFF");
    }
}


//**********************************************
// Function to handle Blynk jog button press
//  for JT4 Reverse
//  
//**********************************************
BLYNK_WRITE(V7)
{
    if (param.asInt()) {
        Serial.printlnf("JT4 reverse ON");
        JointMotors[4].bForward = FALSE;
        digitalWrite(JointMotors[4].dir_pin, JointMotors[4].bForward);
        analogWrite(JointMotors[4].pwm_pin,iJogSpeed,PWM_Freq);
    } else {
        analogWrite(JointMotors[4].pwm_pin,0);
        Serial.printlnf("JT4 reverse OFF");
    }
}


//**********************************************
// Function to handle Blynk jog button press
//  for JT5 Forward
//  
//**********************************************
BLYNK_WRITE(V8)
{
    if (param.asInt()) {
        Serial.printlnf("JT5 forward ON");
        JointMotors[5].bForward = TRUE;
        digitalWrite(JointMotors[5].dir_pin, JointMotors[5].bForward);
        analogWrite(JointMotors[5].pwm_pin,iJogSpeed,PWM_Freq);
    } else {
        analogWrite(JointMotors[5].pwm_pin,0);
        Serial.printlnf("JT5 forward OFF");
    }
}


//**********************************************
// Function to handle Blynk jog button press
//  for JT5 Reverse
//  
//**********************************************
BLYNK_WRITE(V9)
{
    if (param.asInt()) {
        Serial.printlnf("JT5 reverse ON");
        JointMotors[5].bForward = FALSE;
        digitalWrite(JointMotors[5].dir_pin, JointMotors[5].bForward);
        analogWrite(JointMotors[5].pwm_pin,iJogSpeed,PWM_Freq);
    } else {
        analogWrite(JointMotors[5].pwm_pin,0);
        Serial.printlnf("JT5 reverse OFF");
    }
}


//**********************************************
// Function to handle Blynk jog speed
//
//  
//**********************************************
BLYNK_WRITE(V10)
{
    bSerialStayAwake = TRUE;
    iJogSpeed = param.asInt();
    Serial.printlnf("Jog speed:%u",iJogSpeed);
}


//**********************************************
// Function to handle Blynk JT1 target pos
//
//  
//**********************************************
BLYNK_WRITE(V21)
{
    bSerialStayAwake = TRUE;
    comms_in.DATA.Target[1] = param.asInt();
    Serial.printlnf("JT1 target pos:%u",comms_in.DATA.Target[1]);
}


//**********************************************
// Function to handle Blynk JT2 target pos
//
//  
//**********************************************
BLYNK_WRITE(V22)
{
    bSerialStayAwake = TRUE;
    comms_in.DATA.Target[2] = param.asInt();
    Serial.printlnf("JT2 target pos:%u",comms_in.DATA.Target[2]);
}


//**********************************************
// Function to handle Blynk JT3 target pos
//
//  
//**********************************************
BLYNK_WRITE(V23)
{
    bSerialStayAwake = TRUE;
    comms_in.DATA.Target[3] = param.asInt();
    Serial.printlnf("JT3 target pos:%u",comms_in.DATA.Target[3]);
}


//**********************************************
// Function to handle Blynk JT4 target pos
//
//  
//**********************************************
BLYNK_WRITE(V24)
{
    bSerialStayAwake = TRUE;
    comms_in.DATA.Target[4] = param.asInt();
    Serial.printlnf("JT4 target pos:%u",comms_in.DATA.Target[4]);
}


//**********************************************
// Function to handle Blynk JT5 target pos
//
//  
//**********************************************
BLYNK_WRITE(V25)
{
    bSerialStayAwake = TRUE;
    comms_in.DATA.Target[5] = param.asInt();
    Serial.printlnf("JT5 target pos:%u",comms_in.DATA.Target[5]);
}


//**********************************************
// Function to handle Blynk JT1 Reset Pos
//
//  
//**********************************************
BLYNK_WRITE(V26)
{
    bSaveEEPROM = TRUE;
    if (param.asInt()) {
        Joint_pos[1] = 8389; // 8389 is centre position
        reseti2c_encoder(1, 8389);
        comms_out.DATA.JointPos[1] = 8389;
        Serial.printlnf("JT1 posistion reset");
    }
}


//**********************************************
// Function to handle Blynk JT2 Reset Pos
//
//  
//**********************************************
BLYNK_WRITE(V27)
{
    bSaveEEPROM = TRUE;
    if (param.asInt()) {
        Joint_pos[2] = 8389; // 8389 is centre position
        reseti2c_encoder(2, 8389);
        comms_out.DATA.JointPos[2] = 8389;
        Serial.printlnf("JT2 posistion reset");
    }
}


//**********************************************
// Function to handle Blynk JT3 Reset Pos
//
//  
//**********************************************
BLYNK_WRITE(V28)
{
    bSaveEEPROM = TRUE;
    if (param.asInt()) {
        Joint_pos[3] = 8389; // 8389 is centre position
        reseti2c_encoder(3, 8389);
        comms_out.DATA.JointPos[3] = 8389;
        Serial.printlnf("JT3 posistion reset");
    }
}


//**********************************************
// Function to handle Blynk JT4 Reset Pos
//
//  
//**********************************************
BLYNK_WRITE(V29)
{
    bSaveEEPROM = TRUE;
    if (param.asInt()) {
        Joint_pos[4] = 8389; // 8389 is centre position
        reseti2c_encoder(4, 8389);
        comms_out.DATA.JointPos[4] = 8389;
        Serial.printlnf("JT4 posistion reset");
    }
}


//**********************************************
// Function to handle Blynk JT5 Reset Pos
//
//  
//**********************************************
BLYNK_WRITE(V30)
{
    bSaveEEPROM = TRUE;
    if (param.asInt()) {
        Joint_pos[5] = 0; // joint 5 is linear, zero is fully open
        comms_out.DATA.JointPos[5] = 0;
        Serial.printlnf("JT5 posistion reset");
    }
}


//**********************************************
// Function to handle Blynk JT1 Set Max Travel
//
//  
//**********************************************
BLYNK_WRITE(V31)
{
    bSaveEEPROM = TRUE;
    if (param.asInt()) {
        JointMotors[1].dMax_Trav = comms_out.DATA.JointPos[1];
        Serial.printlnf("JT1 set maximum travel");
    }
}


//**********************************************
// Function to handle Blynk JT2 Set Max Travel
//
//  
//**********************************************
BLYNK_WRITE(V32)
{
    bSaveEEPROM = TRUE;
    if (param.asInt()) {
        JointMotors[2].dMax_Trav = comms_out.DATA.JointPos[2];
        Serial.printlnf("JT2 set maximum travel");
    }
}


//**********************************************
// Function to handle Blynk JT3 Set Max Travel
//
//  
//**********************************************
BLYNK_WRITE(V33)
{
    bSaveEEPROM = TRUE;
    if (param.asInt()) {
        JointMotors[3].dMax_Trav = comms_out.DATA.JointPos[3];
        Serial.printlnf("JT3 set maximum travel");
    }
}


//**********************************************
// Function to handle Blynk JT4 Set Max Travel
//
//  
//**********************************************
BLYNK_WRITE(V34)
{
    bSaveEEPROM = TRUE;
    if (param.asInt()) {
        JointMotors[4].dMax_Trav = comms_out.DATA.JointPos[4];
        Serial.printlnf("JT4 set maximum travel");
    }
}


//**********************************************
// Function to handle Blynk JT5 Set Max Travel
//
//  
//**********************************************
BLYNK_WRITE(V35)
{
    bSaveEEPROM = TRUE;
    if (param.asInt()) {
        JointMotors[5].dMax_Trav = comms_out.DATA.JointPos[5];
        Serial.printlnf("JT5 set maximum travel");
    }
}


//**********************************************
// Function to handle saving to EEPROM
//
//  
//**********************************************
void HandleSave(void)
{
    iEEPROM_Ptr = 1;
    JointMotors[1].dPosition = comms_out.DATA.JointPos[1];
    EEPROM.put(iEEPROM_Ptr, JointMotors[1]);
    Serial.printlnf("Write JT1 Pos:%u to EEPROM:%u",JointMotors[1].dPosition, iEEPROM_Ptr);
    iEEPROM_Ptr = iEEPROM_Ptr + sizeof(JointMotors) + 5;
    JointMotors[2].dPosition = comms_out.DATA.JointPos[2];
    EEPROM.put(iEEPROM_Ptr, JointMotors[2]);
    Serial.printlnf("Write JT2 Pos:%u to EEPROM:%u",JointMotors[2].dPosition, iEEPROM_Ptr);
    iEEPROM_Ptr = iEEPROM_Ptr + sizeof(JointMotors) + 5;
    JointMotors[3].dPosition = comms_out.DATA.JointPos[3];
    EEPROM.put(iEEPROM_Ptr, JointMotors[3]);
    Serial.printlnf("Write JT3 Pos:%u to EEPROM:%u",JointMotors[3].dPosition, iEEPROM_Ptr);
    iEEPROM_Ptr = iEEPROM_Ptr + sizeof(JointMotors) + 5;
    JointMotors[4].dPosition = comms_out.DATA.JointPos[4];
    EEPROM.put(iEEPROM_Ptr, JointMotors[4]);
    Serial.printlnf("Write JT4 Pos:%u to EEPROM:%u",JointMotors[4].dPosition, iEEPROM_Ptr);
    iEEPROM_Ptr = iEEPROM_Ptr + sizeof(JointMotors) + 5;
    JointMotors[5].dPosition = comms_out.DATA.JointPos[5];
    EEPROM.put(iEEPROM_Ptr, JointMotors[5]);
    Serial.printlnf("Write JT5 Pos:%u from EEPROM:%u",JointMotors[5].dPosition, iEEPROM_Ptr);
    
    // read it back as a check
    Serial.printlnf("Read version:%u",EEPROM.read(0));
    iEEPROM_Ptr = 1;
    EEPROM.get(iEEPROM_Ptr, JointMotors[1]);
    comms_out.DATA.JointPos[1] = JointMotors[1].dPosition;
    Serial.printlnf("Read JT1 Pos:%u from EEPROM:%u",JointMotors[1].dPosition, iEEPROM_Ptr);
}


//**********************************************
// Function to handle reading char from serial
//  Non Blocking, reads to buffer until CR terminator
//  
//**********************************************
void HandleSerial(void)
{
    // read from port 0 and store in buffer
    if(Serial.available()){
        cBuffer[iBufferPtr] = Serial.read();
        // check for CR
        if(cBuffer[iBufferPtr] == 13){
            bStringFound =  TRUE;
            iBufferPtr=0;
        }
        else{
            ++iBufferPtr;
        }
    }
    // check for CR
    if(bStringFound){
        bStringFound = FALSE;
        //sBuffer.toUpperCase();
        // extract joint number
        //Serial.printlnf("buffer:%s",String(cBuffer));

        iRequestType = cBuffer[0];
        cBuffer[0] = '0';
        iRequestJoint = cBuffer[1] - 48;
        cBuffer[1] = '0';
        sBuffer = String(cBuffer);
        
        if(iRequestType != 'W' && iRequestType != 'R' && iRequestType != 'B' && iRequestType != 'F' && iRequestType != 'Z' && iRequestType != 'Y' && iRequestType != 'I' && (iRequestType>57) && !(iRequestJoint > 0 && iRequestJoint < 6)){
            iRequestType = 0;
            iRequestJoint = 0;
            Serial.printlnf("Invalid joint number");
        }
        
        // Check the first char for request type
        switch(iRequestType)
        {
            case 'P':
            {
                Serial.printlnf("JT%u position:%u",iRequestJoint,comms_out.DATA.JointPos[iRequestJoint]);
            } break;

            case 'S':
            {
                if(sBuffer.toFloat() > 255){
                    comms_in.DATA.Speed[iRequestJoint] = 255;
                }
                else{
                    comms_in.DATA.Speed[iRequestJoint] = sBuffer.toInt();
                }
                Serial.printlnf("JT%u speed:%u",iRequestJoint,comms_in.DATA.Speed[iRequestJoint]);
            } break;

            case 'T':
            {
                comms_in.DATA.Target[iRequestJoint] = int(sBuffer.toFloat());
                Serial.printlnf("JT%u target:%u",iRequestJoint,comms_in.DATA.Target[iRequestJoint]);
            } break;

            case 'M':
            {
                Serial.printlnf("JT%u max travel:%u",iRequestJoint,JointMotors[iRequestJoint].dMax_Trav);
            } break;

            case 'W':
            {
                bSerialStayAwake = TRUE;
            } break;

            case 'R':
            {
                bSaveEEPROM = FALSE;
                HandleSave();
                Serial.printlnf("Reset photon");
            	delay(1000);
                System.sleep(SLEEP_MODE_DEEP,1);
            } break;

            case 'C':
            {
                bSerialStayAwake = TRUE;
                JointMotors[iRequestJoint].bCalibrate = TRUE;
                iCalSpeed = sBuffer.toInt();
                Serial.printlnf("JT%u calibrate at %u",iRequestJoint, iCalSpeed);
            } break;

            case 'B':
            {
                bSaveEEPROM = TRUE;
                Serial.printlnf("Save all positions and max travels");
                Serial.printlnf("JT1 max PWM freq %u",analogWriteMaxFrequency(JointMotors[1].pwm_pin));
                Serial.printlnf("JT2 max PWM freq %u",analogWriteMaxFrequency(JointMotors[2].pwm_pin));
                Serial.printlnf("JT3 max PWM freq %u",analogWriteMaxFrequency(JointMotors[3].pwm_pin));
                Serial.printlnf("JT4 max PWM freq %u",analogWriteMaxFrequency(JointMotors[4].pwm_pin));
                Serial.printlnf("JT5 max PWM freq %u",analogWriteMaxFrequency(JointMotors[5].pwm_pin));
            } break;

            case 'F':
            {
                EEPROM.write(0,0xFF);
                Serial.printlnf("Factory default all saved data");
            	delay(1000);
                System.sleep(SLEEP_MODE_DEEP,1);
            } break;
            
            case 'I':
            {
                Serial.printlnf("WiFi local IP:");
                Serial.println(WiFi.localIP());
            } break;
            
            case 'Y':
            {
                bROS_Control = TRUE;
                bROS_Control_Vel = TRUE;
                Serial.printlnf("Y");
            } break;
            
            case 'Z':
            {
                bROS_Control = TRUE;
                Serial.printlnf("Z");
            } break;
            
            default:
            {
                Serial.printlnf("Invalid request");
                iRequestType = 0;
            } break;
        }
        iRequestType = 0;
        iRequestJoint = 0;
    }
}


//**********************************************
// Drives joint in reverse until current overload
// Then drives forward until current overload
// Timing travel time
//**********************************************
void CalibrateMotor(int joint)
{
	switch(iCalJoint)
	{
		case 0: // not needed, spare step
		{
			iCalJoint++;
		}break;

		case 1: // Prepaid to drive in reverse
		{
			JointMotors[joint].bForward = FALSE;
			digitalWrite(JointMotors[joint].dir_pin, JointMotors[joint].bForward);
			analogWrite(JointMotors[joint].pwm_pin,iCalSpeed,PWM_Freq);
			comms_out.DATA.JointPos[joint] = 5000;
			MotorStalled.reset();
			iCalJoint++;
			Serial.printlnf("Step1: JT%u begin reverse at %u",joint, iCalSpeed);
			bMotorStalled = FALSE;
		}break;

		case 2: // Wait for the motor start turning, or maybe its already fully reversed
		{
			if(comms_out.DATA.JointPos[joint] != lCalPosPre)
			{
				MotorStalled.reset();
				bMotorStalled = FALSE;
				lCalPosPre = comms_out.DATA.JointPos[joint];
	        }
		    if(bMotorStalled)
			{
				Serial.printlnf("Step2: JT%u stalled rev",joint);
				bMotorStalled = FALSE;
				iCalJoint++;
		    }
		}break;

		case 3:	// Drive motor forward until Stalled
		{
			Serial.printlnf("Step3: JT%u drive fwd",joint);
			JointMotors[joint].bForward = TRUE;
			digitalWrite(JointMotors[joint].dir_pin, JointMotors[joint].bForward);
			analogWrite(JointMotors[joint].pwm_pin,iCalSpeed,PWM_Freq);
			comms_out.DATA.JointPos[joint]=0;
			lCalPosPre = comms_out.DATA.JointPos[joint];
			MotorStalled.reset();
			iCalJoint++;
			bMotorStalled = FALSE;
		}break;

		case 4: // Wait for the motor start turning, or maybe its already fully reversed
		{
			if(comms_out.DATA.JointPos[joint] != lCalPosPre)
			{
				MotorStalled.reset();
				bMotorStalled = FALSE;
				lCalPosPre = comms_out.DATA.JointPos[joint];
	        }
		    if(bMotorStalled)
			{
				Serial.printlnf("Step4: JT%u stalled fwd",joint);
				bMotorStalled = FALSE;
				iCalJoint++;
		    }
		}break;

		case 5: // Instruct motor to drive to joint target of mid position
		{
		    comms_in.DATA.Target[joint] = comms_out.DATA.JointPos[joint]/2;
		    comms_in.DATA.Speed[joint] = iCalSpeed;
            Serial.printlnf("Step5: JT%u finished, full trav:%u , target:%u",iJointCycle,comms_out.DATA.JointPos[joint],comms_in.DATA.Target[joint]);
            analogWrite(JointMotors[joint].pwm_pin,0);
            JointMotors[joint].dMax_Trav = comms_out.DATA.JointPos[joint];
            bSaveEEPROM = TRUE;
    		JointMotors[joint].bCalibrate = FALSE;
			iCalJoint=0;
		}break;

		default:
		{
			iCalJoint=0;
		}break;
	}
}


//**********************************************
// Function to control the motor joint positions
//  0 to xxxx counts
//  convert to 0 to xxx degree
//**********************************************
void HandleJoints(int joint)
{
    if(joint < 5)
    {
        comms_out.DATA.JointPos[joint] = readi2c_encoder(joint);
    }
    else
    {
        comms_out.DATA.JointPos[5] = Joint_pos[5];
    }
    
	if((comms_in.DATA.Target[joint] >= (comms_out.DATA.JointPos[joint] - iTargetRange)) && (comms_in.DATA.Target[joint] <= (comms_out.DATA.JointPos[joint] + iTargetRange)))
	{
        analogWrite(JointMotors[joint].pwm_pin,0);
        iTargetRange = 1; // once we reach our intended target then increase our target
	}
	else
	{
        //Serial.printlnf("JT%u pos:%u  target:%u",iJointCycle,comms_out.DATA.JointPos[joint],comms_in.DATA.Target[joint]);
        //delay(500);
        analogWrite(JointMotors[joint].pwm_pin,comms_in.DATA.Speed[joint],PWM_Freq);
        iTargetRange = 0;
    	if(comms_out.DATA.JointPos[joint] > comms_in.DATA.Target[joint])
    	{
            JointMotors[joint].bForward = FALSE;
        }
        else
    	{
            JointMotors[joint].bForward = TRUE;
        }
        digitalWrite(JointMotors[joint].dir_pin, JointMotors[joint].bForward);
	}
}


//**********************************************
// Function to control the motor joint velocities
//  0 to 128 to 255 valve
//
//    0 to 127 is reverse
//         128 is stopped
//  129 to 255 is forward
//**********************************************
void HandleJointsVel(int joint)
{
    uint8_t joint_speed;
    
    if(joint < 5)
    {
        comms_out.DATA.JointPos[joint] = readi2c_encoder(joint);
    }
    else
    {
        comms_out.DATA.JointPos[5] = Joint_pos[5];
    }
    
	if(comms_in.DATA.Speed[joint] == 128)
	{
        analogWrite(JointMotors[joint].pwm_pin,0);
	}
	else
	{
        if(comms_in.DATA.Speed[joint] > 128)
        {
            joint_speed = (comms_in.DATA.Speed[joint] - 128) * 2;
            JointMotors[joint].bForward = TRUE;
        }
        else
    	{
    	    if(comms_in.DATA.Speed[joint] > 0)
    	    {
                joint_speed = (128 - comms_in.DATA.Speed[joint]) * 2;
    	    }
    	    else
    	    {
    	        joint_speed = 255;
    	    }
            JointMotors[joint].bForward = FALSE;
        }
        analogWrite(JointMotors[joint].pwm_pin,joint_speed,PWM_Freq);
        digitalWrite(JointMotors[joint].dir_pin, JointMotors[joint].bForward);
	}
}


