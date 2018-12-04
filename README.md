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

 This code implements MIDI over Bluetooth Low Energy.
 Its a modification of Neilbags demo app incorporating changes from https://github.com/nkolban/esp32-snippets/issues/510
 I left out the BT encryption stuff.

 The code creates a BLE MIDI service and characteristic, advertises the service and waits for a client connection.
 It converts note on/off messages to gate outs on the first row of 4 output jacks and control voltages on last row of 4 jacks.
 
 MIDI channel 1 = GATE0 +CV0
 
 MIDI channel 2 = GATE1 +CV1
 
 MIDI channel 3 = GATE2 +CV2
 
 MIDI channel 4 = GATE3 +CV3
 
 Trigger outputs are on 2nd row of 4 jacks. Triggers are sent on MIDI channel 10 which is by convention for drums. Trigger channel number is note number mod 4 
  ie MIDI note 0=trigger0, note 1=trigger1, note 2=trigger2, note 3 =trigger3, note4=trigger0 etc 

 TBD:
 implement more MIDI commands
 add a menu system for configuration

