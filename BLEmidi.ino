/*
 Bluetooth Low Energy MIDI interface for Eurorack using ESP32 DEV module
 R Heslip Nov 2018 

 The hardware module this code runs on is along the lines of Ardcore but much more powerful and versatile. The ESP32 supports WiFi and Bluetooth Low Energy.
 Hardware includes a small SSD1306 128x32 I2C OLED display, an encoder and encoder switch for menus (menus system not currently implemented)
 4 LEDs
 4 GATE output jacks
 4 Trigger output jacks
 4 CV output jacks
 2X MCP4822 12 2 channel DACs are used for the CV outs. They run off the ESP32 module's 3.3v power supply so we use the 2.048V internal reference in the MCP4822
 I used ~ 1khz RC lowpass filters on the DAC outputs to deglitch and as an antialising filter for using the module as an LFO, ADSR etc
 a TL072 quad opamp buffers the filtered DAC outs with a gain of 3 which gives a CV output range of 0-6.144V - a bit over 6 octaves
 the GPIOs can be used as inputs or outputs and the first two on the second row can be used as CV ins
 protection for the GPIO pins is MANDATORY. I used a 1k current limiter from the jack to a BAT54 dual schottky diode which clamps the GPIOs to GND and 3.3V
 There is another 1k resistor from the diode clamp to the ESP32 pins for more protection against latchup. The down side to using 3.3v GPIOs directly is some modules may not work with 3.3v signal levels
 and because the pins are clamped to GND and 3.3v it will load signals that exceed these voltages a lot more than a typical 100k impedance input.

 This code implements MIDI over Bluetooth Low Energy
 its a modification of Neilbags demo app incorporating changes from https://github.com/nkolban/esp32-snippets/issues/510
 I left out the BT encryption stuff

 The code creates a BLE MIDI service and characteristic, advertises the service and waits for a client connection
 converts note on/off messages to gate outs on the first row of 4 output jacks and control voltages on last row of 4 jacks
 MIDI channel 1 = GATE0 +CV0
 MIDI channel 2 = GATE1 +CV1
 MIDI channel 3 = GATE2 +CV2
 MIDI channel 4 = GATE3 +CV3
 Trigger outputs are on 2nd row of 4 jacks. Triggers are sent on MIDI channel 10 which is by convention for drums. Trigger channel number is note number mod 4 
  ie MIDI note 0=trigger0, note 1=trigger1, note 2=trigger2, note 3 =trigger3, note4=trigger0 etc 

 TBD:
 implement more MIDI commands
 add a menu system for configuration
 
 Dec 1/18 - incorporated BLEmidi decoder from the Pedalino project which resolved problem of notes getting dropped
 dec 3/18 - restructured the code to use channel numbers for LEDs,DACs and GPIOs  
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Wire.h>
#include "SPI.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// create display device
#define OLED_RESET 2  // unused port - to keep the compiler happy
Adafruit_SSD1306 display(OLED_RESET);

// DAC uses 2.048v vref and is amplified 3x ie 6.144V full scale 
// the DACs/amps are not perfect and really should be calibrated in software
// 6v is 6 octaves so each note step is 4000/(6*12)=55.5555 counts. if we use 56 there is some error but its OK for a test
#define NOTE_STEP 56
#define LOWEST_NOTE 24 // make C1 the lowest MIDI note ie 0V output
#define HIGHEST_NOTE 96 // highest MIDI note C7 ie 6V output

// DAC chip selects
#define DAC0_CS 17
#define DAC1_CS 16

// SPI stuff
#define MISO 12
#define MOSI 13
#define SCLK 14


// LED output pins
#define LED0 5
#define LED1 18
#define LED2 19
#define LED3 2

// GPIO pins - these can be input, output or analog input
// 25 and 26 are on A/D 2 which apparently doesn't work when wifi is active
#define GPIO_0 25
#define GPIO_1 26
#define GPIO_2 4
#define GPIO_3 27
#define GPIO_4 32
#define GPIO_5 33
#define GPIO_6 0
#define GPIO_7 15

// encoder pins
#define ENC_A   34
#define ENC_B   39
#define ENC_SW  35

// dac_write - send a 12 bit value to the DAC
// there are 2 DACs, each with 2 channels of output
// channels 0,1 on first chip, 2,3 on 2nd chip
// channel = channel number 0-3
// value - 12 bit DAC value to write

void dac_write(int channel, unsigned value) {
  if (channel &2) digitalWrite(DAC1_CS, LOW); // assert correct chip select
  else digitalWrite(DAC0_CS, LOW);
  byte data = value >> 8;  // get to 8 bits of DAC value
  data = data & B00001111; // mask off 4 msbs which are control
  if (channel &1) data = data | B10110000; //2nd DAC channel, 2.048v mode
  else data = data | B00110000; //1st DAC channel, 2.048v mode
  SPI.transfer(data); // send  control bits and 4 MSBs of DAC value
  data = value;
  SPI.transfer(data);  // send low 8 bits of DAC value
  if (channel &2) digitalWrite(DAC1_CS, HIGH);
  else digitalWrite(DAC0_CS, HIGH);
}

// write value to LEDs 0-3
// 
void LED_write(int channel, bool value) {
  switch (channel) {
    case 0:
       digitalWrite(LED0,value); // 
       break;
    case 1:
       digitalWrite(LED1,value); // 
       break;
    case 2:
       digitalWrite(LED2,value); // 
       break;
    case 3:
       digitalWrite(LED3,value); // 
       break;
  }
}

// write value to GPIO 0-7 - gate and trigger outputs
//  

void GPIO_write(int channel, bool value) {
  switch (channel) {
    case 0:
       digitalWrite(GPIO_0,value); // 
       break;
    case 1:
       digitalWrite(GPIO_1,value); // 
       break;
    case 2:
       digitalWrite(GPIO_2,value); // 
       break;
    case 3:
       digitalWrite(GPIO_3,value); // 
       break;
    case 4:
       digitalWrite(GPIO_4,value); // 
       break;
    case 5:
       digitalWrite(GPIO_5,value); // 
       break;
    case 6:
       digitalWrite(GPIO_6,value); // 
       break;
    case 7:
       digitalWrite(GPIO_7,value); // 
       break;
  }
}

// bluetooth stuff

#define SERVICE_UUID        "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

// incoming midi packet handler

class MyCharacteristicCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pChar){
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
          BLEMidiReceive((uint8_t *)(rxValue.c_str()), rxValue.length());
          //Serial.print("BLE Received %2d bytes", rxValue.length());  
      }
    }
};

// decode MIDI commands and update the LEDs, Gates, DACs

void MIDIaction(unsigned char command, unsigned char data1, unsigned char data2, int channel) {

      if ((command==0x90) && (data2==0)) command=0x80; // sometimes note on with velocity 0 is used as note off - if so change the command to note off
      if ((data1 >= LOWEST_NOTE) &&( data1 <= HIGHEST_NOTE)) { // don't play notes out of DAC range
        unsigned dacval=(data1-LOWEST_NOTE)*NOTE_STEP;
        // Serial.println(dacval);
        switch (command) { // midi command dispatch
          case 0x90:    // note on msg
            if (channel == 9) { // channel 9 (ie MIDI channel 10) is used for triggers on 2nd row of GPIO jacks
              GPIO_write(4+(data1%4),1); // triggers are on 2nd row GPIOs. trigger number is note number mod 4 
            }
            else {
              LED_write(channel,1); // turn on the LED
              GPIO_write(channel,1); //turn on the gate
              dac_write(channel,dacval);  // set the CV
           }
          break;  
          case 0x80:    // note off msg
            if (channel == 9) { // channel 9 (ie MIDI channel 10) is used for triggers on 2nd row of GPIO jacks
              GPIO_write(4+(data1%4),0); // triggers are on 2nd row GPIOs. trigger number is note number mod 4  
            }
            else {
              LED_write(channel,0); // turn off the LED
              GPIO_write(channel,0); //turn off the gate
           }
          break;
        }
      }
}

// Decodes the BLE characteristics and calls MIDIaction if the packet contains actionable MIDI data
// https://learn.sparkfun.com/tutorials/midi-ble-tutorial

void BLEMidiReceive(uint8_t *buffer, uint8_t bufferSize)
{
  /*
    The general form of a MIDI message follows:

    n-byte MIDI Message
      Byte 0            MIDI message Status byte, Bit 7 is Set to 1.
      Bytes 1 to n-1    MIDI message Data bytes, if n > 1. Bit 7 is Set to 0
    There are two types of MIDI messages that can appear in a single packet: full MIDI messages and
    Running Status MIDI messages. Each is encoded differently.
    A full MIDI message is simply the MIDI message with the Status byte included.
    A Running Status MIDI message is a MIDI message with the Status byte omitted. Running Status
    MIDI messages may only be placed in the data stream if the following criteria are met:
    1.  The original MIDI message is 2 bytes or greater and is not a System Common or System
        Real-Time message.
    2.  The omitted Status byte matches the most recently preceding full MIDI message’s Status
        byte within the same BLE packet.
    In addition, the following rules apply with respect to Running Status:
    1.  A Running Status MIDI message is allowed within the packet after at least one full MIDI
        message.
    2.  Every MIDI Status byte must be preceded by a timestamp byte. Running Status MIDI
        messages may be preceded by a timestamp byte. If a Running Status MIDI message is not
        preceded by a timestamp byte, the timestamp byte of the most recently preceding message
        in the same packet is used.
    3.  System Common and System Real-Time messages do not cancel Running Status if
        interspersed between Running Status MIDI messages. However, a timestamp byte must
        precede the Running Status MIDI message that follows.
    4.  The end of a BLE packet does cancel Running Status.
    In the MIDI 1.0 protocol, System Real-Time messages can be sent at any time and may be
    inserted anywhere in a MIDI data stream, including between Status and Data bytes of any other
    MIDI messages. In the MIDI BLE protocol, the System Real-Time messages must be deinterleaved
    from other messages – except for System Exclusive messages.
  */
  int   channel;
  unsigned char  command;

  //Pointers used to search through payload.
  uint8_t lPtr = 0;
  uint8_t rPtr = 0;
  //Decode first packet -- SHALL be "Full MIDI message"
  lPtr = 2; //Start at first MIDI status -- SHALL be "MIDI status"
  //While statement contains incrementing pointers and breaks when buffer size exceeded.
  while (1) {
    //lastStatus used to capture runningStatus
    uint8_t lastStatus = buffer[lPtr];
    if ( (buffer[lPtr] < 0x80) ) {
      //Status message not present, bail
      return;
    }
    //command = MIDI.getTypeFromStatusByte(lastStatus);
    //channel = MIDI.getChannelFromStatusByte(lastStatus);
    command = lastStatus & 0xf0;
    channel = lastStatus & 0x0f;    
    //Point to next non-data byte
    rPtr = lPtr;
    while ( (buffer[rPtr + 1] < 0x80) && (rPtr < (bufferSize - 1)) ) {
      rPtr++;
    }
    //look at l and r pointers and decode by size.
    if ( rPtr - lPtr < 1 ) {
      //Time code or system
      MIDIaction(command, 0, 0, channel);
    } else if ( rPtr - lPtr < 2 ) {
      MIDIaction(command, buffer[lPtr + 1], 0, channel);
    } else if ( rPtr - lPtr < 3 ) {
      MIDIaction(command, buffer[lPtr + 1], buffer[lPtr + 2], channel);
    } else {
      //Too much data
      //If not System Common or System Real-Time, send it as running status
      switch ( buffer[lPtr] & 0xF0 )
      {
        case 0x80:
        case 0x90:
        case 0xA0:
        case 0xB0:
        case 0xE0:
          for (int i = lPtr; i < rPtr; i = i + 2) {
            MIDIaction(command, buffer[i + 1], buffer[i + 2], channel);
          }
          break;
        case 0xC0:
        case 0xD0:
          for (int i = lPtr; i < rPtr; i = i + 1) {
            MIDIaction(command, buffer[i + 1], 0, channel);
          }
          break;
        default:
          break;
      }
    }
    //Point to next status
    lPtr = rPtr + 2;
    if (lPtr >= bufferSize) {
      //end of packet
      return;
    }
  }
}





void setup() {
  Serial.begin(115200);
//  Serial.println("BLE MIDI");
  
  pinMode(DAC0_CS, OUTPUT);
  pinMode(DAC1_CS, OUTPUT);
  digitalWrite(DAC0_CS, HIGH);
  digitalWrite(DAC1_CS, HIGH);

  SPI.begin(SCLK,MISO,MOSI,DAC0_CS); // we actually use a CS pin for each DAC
  SPI.setBitOrder(MSBFIRST);
  
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(GPIO_0, OUTPUT); // trigger/gate outputs
  pinMode(GPIO_1, OUTPUT);
  pinMode(GPIO_2, OUTPUT);
  pinMode(GPIO_3, OUTPUT);
  pinMode(GPIO_4, OUTPUT);
  pinMode(GPIO_5, OUTPUT);
  pinMode(GPIO_6, OUTPUT);
  pinMode(GPIO_7, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);  

  SPI.begin(SCLK,MISO,MOSI,DAC0_CS); // we actually use a CS pin for each DAC
  SPI.setBitOrder(MSBFIRST);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.clearDisplay();
  display.println("BLEMIDI 12/3/18");
  display.display();

  BLEDevice::init("ESP32");
   Serial.println("BLE device created");   
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
     Serial.println("BLE server created");  
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
 // BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));
  BLEService* pService = pServer->createService("03b80e5a-ede8-4b33-a751-6ce34ec4c700");
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic("7772e5db-3868-4112-a1a9-f2669d106bf3", 
                    BLECharacteristic::PROPERTY_READ   |
                    BLECharacteristic::PROPERTY_NOTIFY |
                    BLECharacteristic::PROPERTY_WRITE_NR
        );
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml

  pCharacteristic->setCallbacks(new MyCharacteristicCallback());
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();
//  pCharacteristic->setValue(midiPacket, 5); // storage for incoming packet, length in bytes
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->start();
  // Start advertising
 // pServer->getAdvertising()->start();

  display.setCursor(24,16);
  display.print("Not Connected  ");
  display.display();
  digitalWrite(LED3, LOW);  // turn off the BT led
  Serial.println("starting Loop");  
}

bool connectedstatus=false;  // flag to keep track of status display

void loop() {
  
  if (deviceConnected) {
    if (connectedstatus==false) {  // do only on connection so we don't chew up CPU time
      display.setTextColor(BLACK);
      display.setCursor(24,16);
      display.print("Not Connected  ");// driver seems to OR draw pixels so we have to undraw last text
      display.setCursor(24,16);
      display.setTextColor(WHITE);
      display.print("Connected    ");
      display.display();
      Serial.println("Connected");  
      connectedstatus=true;
    }
  } 
  else {
      if (connectedstatus==true) {
        display.setCursor(24,16);
        display.setTextColor(BLACK);
        display.print("Connected    "); // driver seems to OR draw pixels so we have to undraw
        display.setTextColor(WHITE);
        display.setCursor(24,16);
        display.print("Not Connected  ");
        display.display();
        Serial.println("Not Connected");  
        connectedstatus=false;
      }
   }
/*   Serial.println("sending note");
   // note down
   midiPacket[2] = 0x90; // note down, channel 0
   midiPacket[4] = 127;  // velocity
   pCharacteristic->setValue(midiPacket, 5); // packet, length in bytes
   pCharacteristic->notify();

   play note for 500ms
   delay(500);

   // note up
   midiPacket[2] = 0x80; // note up, channel 0
   midiPacket[4] = 0;    // velocity
   pCharacteristic->setValue(midiPacket, 5); // packet, length in bytes)
   pCharacteristic->notify();

   delay(500);
   */
}
