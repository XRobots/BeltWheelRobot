#include <Arduino.h>
#include "ODriveCAN.h"
#include <Wire.h>

#include <IBusBM.h>
IBusBM IBus; // IBus object

// include the library for the AS5047P sensor.
#include <AS5047P.h>
// define the chip select port.
#define AS5047P_CHIP_SELECT_PORT 10
// define the spi bus speed 
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000
// initialize a new AS5047P sensor object.
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

// ODrive node_id for odrvives
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1
// Uncomment below the line that corresponds to your hardware.
// See also "Board-specific settings" to adapt the details for your hardware setup.

#define IS_TEENSY_BUILTIN // Teensy boards with built-in CAN interface (e.g. Teensy 4.1). See below to select which interface to use.
// #define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
// #define IS_MCP2515 // Any board with external MCP2515 based extension module. See below to configure the module.

/* Board-specific includes ---------------------------------------------------*/

#if defined(IS_TEENSY_BUILTIN) + defined(IS_ARDUINO_BUILTIN) + defined(IS_MCP2515) != 1
#warning "Select exactly one hardware option at the top of this file."

#if CAN_HOWMANY > 0 || CANFD_HOWMANY > 0
#define IS_ARDUINO_BUILTIN
#warning "guessing that this uses HardwareCAN"
#else
#error "cannot guess hardware version"
#endif

#endif

#ifdef IS_ARDUINO_BUILTIN
// See https://github.com/arduino/ArduinoCore-API/blob/master/api/HardwareCAN.h
// and https://github.com/arduino/ArduinoCore-renesas/tree/main/libraries/Arduino_CAN

#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#endif // IS_ARDUINO_BUILTIN

#ifdef IS_MCP2515
// See https://github.com/sandeepmistry/arduino-CAN/
#include "MCP2515.h"
#include "ODriveMCPCAN.hpp"
#endif // IS_MCP2515

# ifdef IS_TEENSY_BUILTIN
// See https://github.com/tonton81/FlexCAN_T4
// clone https://github.com/tonton81/FlexCAN_T4.git into /src
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // hack to prevent teensy compile error
#endif // IS_TEENSY_BUILTIN

/* Board-specific settings ---------------------------------------------------*/

/* Teensy */

#ifdef IS_TEENSY_BUILTIN

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

#endif // IS_TEENSY_BUILTIN

/* MCP2515-based extension modules -*/

#ifdef IS_MCP2515

MCP2515Class& can_intf = CAN;

// chip select pin used for the MCP2515
#define MCP2515_CS 10

// interrupt pin used for the MCP2515
// NOTE: not all Arduino pins are interruptable, check the documentation for your board!
#define MCP2515_INT 2

// freqeuncy of the crystal oscillator on the MCP2515 breakout board. 
// common values are: 16 MHz, 12 MHz, 8 MHz
#define MCP2515_CLK_HZ 8000000


static inline void receiveCallback(int packet_size) {
  if (packet_size > 8) {
    return; // not supported
  }
  CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
  CAN.readBytes(msg.buffer, packet_size);
  onCanMessage(msg);
}

bool setupCan() {
  // configure and initialize the CAN bus interface
  CAN.setPins(MCP2515_CS, MCP2515_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);
  if (!CAN.begin(CAN_BAUDRATE)) {
    return false;
  }

  CAN.onReceive(receiveCallback);
  return true;
}

#endif // IS_MCP2515


/* Arduinos with built-in CAN */

#ifdef IS_ARDUINO_BUILTIN

HardwareCAN& can_intf = CAN;

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

#endif

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0, &odrv1}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData0 {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

struct ODriveUserData1 {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData0 odrv0_user_data;
ODriveUserData1 odrv1_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data_ptr) {

ODriveUserData0* odrv_user_data0 = static_cast<ODriveUserData0*>(user_data_ptr);
odrv_user_data0->last_heartbeat = msg;
odrv_user_data0->received_heartbeat = true;

ODriveUserData1* odrv_user_data1 = static_cast<ODriveUserData1*>(user_data_ptr);
odrv_user_data1->last_heartbeat = msg;
odrv_user_data1->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data_ptr) {
ODriveUserData0* odrv_user_data0 = static_cast<ODriveUserData0*>(user_data_ptr);
odrv_user_data0->last_feedback = msg;
odrv_user_data0->received_feedback = true;

ODriveUserData1* odrv_user_data1 = static_cast<ODriveUserData1*>(user_data_ptr);
odrv_user_data1->last_feedback = msg;
odrv_user_data1->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

unsigned long currentMillis;
long previousMillis = 0;        // set up timers
long previous2Millis = 0;        // set up timers

float roll;
float pitch;
float yaw;

float trimmerRoll;
float trimmerPitch;

int clFlag = 0;

float pot1;   // trim
float pot2;   // trim
float pot3;   // gain
int sw1;    // init
float angle;    // AS5047 wheel angle
float angleCalibrated;    // AS5047 wheel angle offset etc
float angleDivided0;       // eighth of the whole angle
float angleDivided;       // eighth of the whole angle
float angleMult;          // final multiplier for little wheel velocity

float encoder0;
float encoder1;

float rollHold;

float OdriveOutput1;
float OdriveOutput2;

int ch1;
int ch2;
int ch3;
int ch4;
int ch5;
int ch6;
int ch7;
int ch8;
int ch9;
int ch10;

float RFB;
float RFBa;
float RFBFiltered;
float RLR;
float RLRa;
float RLRFiltered;
float RT;
float RTa;
float RTFiltered;

float LFB;
float LFBa;
float LFBFiltered;
float LLR;
float LLRa;
float LLRFiltered;
float LT;
float LTa;
float LTFiltered;

float drivePitch;
float driveRoll;

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

#define BNO08X_INT  2
#define BNO08X_RST  3
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

//PID
#include <PID_v1.h>

// PID1
double Pk1 = 6.8;
double Ik1 = 50;
double Dk1 = 0.06;

double Setpoint1, Input1, Output1, Output1a;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

// PID2
double Pk2 = 3.0;
double Ik2 = 35;
double Dk2 = 0.01;

double Setpoint2, Input2, Output2, Output2a;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

// PID3 - Roll Pos Hold
double Pk3 = 10;
double Ik3 = 10;
double Dk3 = 0.2;

double Setpoint3, Input3, Output3, Output3a;    // PID variables
PID PID3(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);    // PID Setup

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO08x Read Example");

  Wire.begin();

  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }
  Serial.println("BNO08x found!");
  // Wire.setClock(400000); //Increase I2C data rate to 400kHz
  setReports();

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A6, INPUT);
  pinMode(16, INPUT_PULLUP);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-60, 60);
  PID1.SetSampleTime(10);
  
  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-60, 60);
  PID2.SetSampleTime(10);

  PID3.SetMode(AUTOMATIC);         // pos hold             
  PID3.SetOutputLimits(-0.5, 0.5);
  PID3.SetSampleTime(10);

  // initialize the AS5047P sensor and hold if sensor can't be initialized.
  while (!as5047p.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(5000);
  }

  odrv0.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
  odrv1.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);

  IBus.begin(Serial1, IBUSBM_NOTIMER);    // change to Serial1 or Serial2 port when required
  
  }  // end of setup
  

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form roll, pitch, yaw"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
}

void loop() {     

    if (myIMU.wasReset()) {
      Serial.print("sensor was reset ");
      setReports();
    }
   
    // Has a new event come in on the Sensor Hub Bus?
    if (myIMU.getSensorEvent() == true) {
      pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages
      // is it the correct sensor data we want?
      if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
  
      roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
      pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees    
      }      
    }
           
    if (odrv1_user_data.received_feedback) {
      Get_Encoder_Estimates_msg_t feedback = odrv1_user_data.last_feedback;
      pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages
      odrv1_user_data.received_feedback = false;
      encoder1 = feedback.Pos_Estimate; 
    }

    /// *** TIMED LOOP *** ///   
    
    currentMillis = millis();         
    if (currentMillis - previousMillis >= 10) {  // start timed events    
        previousMillis = currentMillis;

        IBus.loop();

        ch1 = IBus.readChannel(0); // get latest value for servo channel 0
        ch2 = IBus.readChannel(1); // get latest value for servo channel 1
        ch3 = IBus.readChannel(2); // get latest value for servo channel 3
        ch4 = IBus.readChannel(3); // get latest value for servo channel 4
        ch5 = IBus.readChannel(4); // get latest value for servo channel 5
        ch6 = IBus.readChannel(5); // get latest value for servo channel 6
        ch7 = IBus.readChannel(6); // get latest value for servo channel 7
        ch8 = IBus.readChannel(7); // get latest value for servo channel 8
        ch9 = IBus.readChannel(8); // get latest value for servo channel 9
        ch10 = IBus.readChannel(9); // get latest value for servo channel 10
      
        LFB = ch1;
        RFB = ch4;
        RLR = ch2;
        RT = ch6;
        LLR = ch3;
        LT = ch5;
        
        // *** threshold sticks ***
        RFBa = thresholdStick(RFB);
        RLRa = thresholdStick(RLR);
        RTa = thresholdStick(RT);
        LFBa = thresholdStick(LFB);
        LLRa = thresholdStick(LLR);
        LTa = thresholdStick(LT);
      
        // *** filter sticks ***
        RFBFiltered = filter(RFBa, RFBFiltered,10);
        RLRFiltered = filter(RLRa, RLRFiltered,10);
        RTFiltered = filter(RTa, RTFiltered,10);
        LFBFiltered = filter(LFBa, LFBFiltered,10);
        LLRFiltered = filter(LLRa, LLRFiltered,10);
        LTFiltered = filter(LTa, LTFiltered,10);

        drivePitch = RFBFiltered/50;
        driveRoll = RLRFiltered/50; 
        
        pot1 = analogRead(A0);
        pot2 = analogRead(A1);
        pot3 = analogRead(A6);
        sw1 = digitalRead(16);

        if (sw1 == 0 && clFlag == 0) {          // Init Odrives

        //check for ODrives

        Serial.println("Waiting for ODrive 0...");
        while (!odrv0_user_data.received_heartbeat) {
          pumpEvents(can_intf);
          delay(100);
        }
        Serial.println("found ODrive 0");
        
        Serial.println("Waiting for ODrive 1...");
        while (!odrv1_user_data.received_heartbeat) {
          pumpEvents(can_intf);
          delay(100);
        }
        Serial.println("found ODrive 1");  
        for (int i = 0; i < 15; ++i) {
          delay(10);
          pumpEvents(can_intf);
        }        
            Serial.println("Enabling closed loop control 0...");
              while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
              odrv0.clearErrors();
              delay(1);
              odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }
            Serial.println("Enabling closed loop control 1...");
              while (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                odrv1.clearErrors();
                delay(1);
              odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }
            clFlag = 1;          // reset flag
        }

        else if (sw1 == 1) {      // make sure we only power on ODrives once per button press
          clFlag = 0;
        }

        Input1 = pitch;
        Setpoint1 = ((pot1 - 512)/100);// + drivePitch;
        PID1.Compute();

        Input3 = (encoder1/100);      // pos hold     
        Setpoint3 = 0;
        PID3.Compute();

        Input2 = roll;
        Setpoint2 = ((pot2 - 512)/1000);// + driveRoll;
        PID2.Compute();
        
        Serial.println(Setpoint1);
        

        angle = as5047p.readAngleDegree();        // read AS5057
        angle = angle + 4;    // apply offset for magnet positon

        // compensate for offset to angle is always 0 - 360
        
        if (angle > 360) {
            angleCalibrated = angle - 360;
        }
        else {
            angleCalibrated = angle;
        }

        // divide wheel into segments
        if (angleCalibrated <= 45) {
          angleDivided0 = angleCalibrated;         
        }
        if (angleCalibrated > 45 && angleCalibrated <= 90) {
          angleDivided0 = angleCalibrated - 45;         
        }
        else if (angleCalibrated > 90 && angleCalibrated <= 135) {
          angleDivided0 = angleCalibrated - 90;         
        }
        else if (angleCalibrated > 135 && angleCalibrated <= 180) {
          angleDivided0 = angleCalibrated - 135;         
        }
        else if (angleCalibrated > 180 && angleCalibrated <= 225) {
          angleDivided0 = angleCalibrated - 180;         
        }
        else if (angleCalibrated > 225 && angleCalibrated <= 270) {
          angleDivided0 = angleCalibrated - 225;         
        }
        else if (angleCalibrated > 270 && angleCalibrated <= 315) {
          angleDivided0 = angleCalibrated - 270;         
        }
        else if (angleCalibrated > 315 && angleCalibrated <= 360) {
          angleDivided0 = angleCalibrated - 315;         
        }

        // make a multiplication factor to scale the wheel velocity

        if (angleDivided0 > 0 && angleDivided0 <= 22.5) {
          angleDivided = angleDivided0 / 22.5;
        }
        if (angleDivided0 > 22.25 && angleDivided0 <= 45) {
          angleDivided = 2 - (angleDivided0 / 22.5);
        }  

        angleMult = 1 + (angleDivided/3);        

        OdriveOutput1 = Output1; 
        OdriveOutput2 = (Output1 * -1) + ((Output2*6) * angleMult); // differential drive + little wheel velocity mulpiplier

        //OdriveOutput1 = 0; 
        //OdriveOutput2 = ((Output2*6) * angleMult); // differential drive + little wheel velocity mulpiplier

        OdriveOutput1 = OdriveOutput1 * (pot3 /1023); 
        OdriveOutput2 = OdriveOutput2 * (pot3 /1023); 
        odrv0.setVelocity(OdriveOutput1);  
        odrv1.setVelocity(OdriveOutput2);     
        
    } // end of timed loop
        
} // end of main loop
