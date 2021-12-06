/*
  HenHouse V2.0 (c) Loic74 <loic74650@gmail.com> 2020-2021
  HenHouse solar-powered intelligent controller:
  UP/DOWN interface buttons to control Guillotine door opening/closing
  Door has endswitches to stop opening/closing as well as a timeout if endswitches were not reached in time
  Measures temperature in 4 egg-slots + an average of the 4 sensors
  Measures battery voltage as well as charging state (charging, done)
  Goes to DeepSleep until UP/DOWN buttons are pressed or until watchdog wakes it up (every 5 minutes) to broadcast measured inputs data over LoRaWan

  Visit your https://eu1.cloud.thethings.network/console/ device console
  to create an account, and obtain the session keys below which are unique to every board.

  You should then create a file called "arduino_secrets.h", save it in the project folder, and into which you will paste those unique keys in the following form:

  -------------------------------------------------------------------------------------------

  // This EUI must be in little-endian format (lsb), so least-significant-byte
  // first. When copying an EUI from ttnctl output, this means to reverse
  // the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
  // 0x70.
  static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

  // This should also be in little endian format (lsb), see above.
  static const u1_t PROGMEM DEVEUI[8]={ 0x13, 0x05, 0x11, 0x00, 0xXX, 0xXX, 0xXX, 0xXX };
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

  // This key should be in big endian format (msb) (or, since it is not really a
  // number but a block of memory, endianness does not really apply). In
  // practice, a key taken from ttnctl can be copied as-is.
  static const u1_t PROGMEM APPKEY[16] = { 0x0A, 0x60, 0xDE, 0x98, 0x1A, 0x43, 0xC8, 0x1B, 0xF2, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX };
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

  --------------------------------------------------------------------------------------------

***Dependencies and respective revisions used to compile this project***
  https://github.com/PaulStoffregen/OneWire (rev 2.3.4)
  https://github.com/milesburton/Arduino-Temperature-Control-Library (rev 3.7.2)
  https://github.com/bricofoy/yasm (rev 0.9.2)
  https://github.com/mcci-catena/arduino-lmic (rev 4.1.0)
  http://www.airspayce.com/mikem/arduino/RadioHead/ (rev 1.120)
  https://www.arduino.cc/libraries/ArduinoLowPower (rev 1.2.2)
*/

/*
  //TTN decoder function:

  function Decoder(bytes, port) {
  var decoded = {};

  if(port == 3)
  {
    var rawTM = bytes[0] + bytes[1] * 256;
    decoded.Mtemp = sflt162f(rawTM) * 100;

    var rawS1 = bytes[2] + bytes[3] * 256;
    decoded.S1temp = sflt162f(rawS1) * 100;

    var rawS2 = bytes[4] + bytes[5] * 256;
    decoded.S2temp = sflt162f(rawS2) * 100;

    var rawS3 = bytes[6] + bytes[7] * 256;
    decoded.S3temp = sflt162f(rawS3) * 100;

    var rawS4 = bytes[8] + bytes[9] * 256;
    decoded.S4temp = sflt162f(rawS4) * 100;

    var rawlum = bytes[10] + bytes[11] * 256;
    decoded.lum = sflt162f(rawlum) * 100;

    var rawSolum = bytes[12] + bytes[13] * 256;
    decoded.Solum = sflt162f(rawSolum) * 100;

    var rawbat = bytes[14] + bytes[15] * 256;
    decoded.bat = sflt162f(rawbat) * 100;

    var DigInt = (bytes[16] << 8) | bytes[17];

    // Decode digital inputs
    // writeBitmap(false, digitalRead(INTERLCK), digitalRead(SwDOWN), digitalRead(SwUP), digitalRead(STAT2), digitalRead(STAT1), ButtonUPPressed, ButtonDWNPressed);
    decoded.ButtonDown = (DigInt & 1) === 0?0:1;
    decoded.ButtonUp = (DigInt & 2) === 0?0:1;
    decoded.Charging = (DigInt & 4) === 0?0:1;
    decoded.DoneCharging = (DigInt & 8) === 0?0:1;
    decoded.DoorSwUp = (DigInt & 16) === 0?0:1;
    decoded.DoorSwDn = (DigInt & 32) === 0?0:1;
    decoded.DoorIntck = (DigInt & 64) === 0?0:1;
    return decoded;
  }
  else
  if(port == 4)
  {
    // Decode bytes to int
    var DoorWPos = (bytes[0] << 8) | bytes[1];
    var DoorAPos = (bytes[2] << 8) | bytes[3];

    // Decode digital inputs
    decoded.ButtonDown = (DigInt & 1) === 0?0:1;
    decoded.ButtonUp = (DigInt & 2) === 0?0:1;
    decoded.DoorSwUp = (DigInt & 16) === 0?0:1;
    decoded.DoorSwDn = (DigInt & 32) === 0?0:1;
    decoded.DoorIntck = (DigInt & 64) === 0?0:1;
    return decoded;
  }
  }

  function sflt162f(rawSflt16)
  {
  // rawSflt16 is the 2-byte number decoded from wherever;
  // it's in range 0..0xFFFF
  // bit 15 is the sign bit
  // bits 14..11 are the exponent
  // bits 10..0 are the the mantissa. Unlike IEEE format,
  // the msb is transmitted; this means that numbers
  // might not be normalized, but makes coding for
  // underflow easier.
  // As with IEEE format, negative zero is possible, so
  // we special-case that in hopes that JavaScript will
  // also cooperate.
  //
  // The result is a number in the open interval (-1.0, 1.0);
  //
  // throw away high bits for repeatability.
  rawSflt16 &= 0xFFFF;

  // special case minus zero:
  if (rawSflt16 == 0x8000)
    return -0.0;

  // extract the sign.
  var sSign = ((rawSflt16 & 0x8000) !== 0) ? -1 : 1;

  // extract the exponent
  var exp1 = (rawSflt16 >> 11) & 0xF;

  // extract the "mantissa" (the fractional part)
  var mant1 = (rawSflt16 & 0x7FF) / 2048.0;

  // convert back to a floating point number. We hope
  // that Math.pow(2, k) is handled efficiently by
  // the JS interpreter! If this is time critical code,
  // you can replace by a suitable shift and divide.
  var f_unscaled = sSign * mant1 * Math.pow(2, exp1 - 15);

  return f_unscaled;
  }*/
/************************** Configuration ***********************************/

#define SLEEP             ->comment this line to prevent µc from sleeping
//#define DOOR_PROGRESS   ->comment this line to prevent code from broadcasting door % position while opening/closing (saves Lora airtime)
#define DOOR_BUZZ         ->comment this line to prevent buzzer from going off while door opening/closing

//Buzzer PWM pin
#define BUZ 5
int Frequ = 4000;
int Duration = 300;
bool Toggle = 0;

//#define DEBUG           ->comment this line to prevent code from writing debug messages to serial port (must be commented when SLEEP is uncommented)
#include "DebugUtils.h"

// Data logging configuration.
#define LOGGING_FREQ_SECONDS   180       // Seconds to wait before a new sensor reading is logged ans sent over LoRaWan.

#include <SPI.h>
#include "OneWire.h"
#include <DallasTemperature.h>
#include <yasm.h>
#include <Streaming.h>
#include <RH_RF95.h>
#include "ArduinoLowPower.h"
#include <lmic.h>
#include <hal/hal.h>
#include "arduino_secrets.h" //this file should be placed in same folder as the sketch and contains your TTN session keys, see below for more details


// Firmware revision
String Firmw = "2.0";


#define MAX_SLEEP_ITERATIONS   LOGGING_FREQ_SECONDS / 10  // Number of times to sleep (for 10 seconds) before
// a sensor reading is taken and sent to the server.
// Don't change this but rather the variable "LOGGING_FREQ_SECONDS" above.

#define LumThreshold_LOW  5                 //Luminosity threshold to actuate door in the evenings
#define LumThreshold_HIGH 30                //Luminosity threshold to actuate door in the mornings

volatile bool ButtonUPPressed = false;
volatile bool ButtonDWNPressed = false;
volatile bool DoorMoving = false;
volatile bool DoorDWNStateForced = false;
volatile bool DoorUPStateForced = false;
volatile float measuredvlum = LumThreshold_HIGH;
volatile float measuredSolarlum = LumThreshold_HIGH;
volatile int sleepIterations = 0;

// Data wire is connected to input digital pin A4 of the Adafruit Feather 32u4 LoRa
#define ONE_WIRE_BUS_A A4

//Motorized door state machine
static YASM Door;

//Door position
volatile double DoorWantedPos = 100.0;
volatile int DoorActualPos = 0;

//Door movements
#define DoorOPEN     1
#define DoorCLOSE    0
#define InContact    0  //endswitches state when in limit position

//Time required for the door to open + 20% or so. Serves as timeout in case endswitches don't work
const unsigned long DoorTimeConstant = 60000L; //60 sec
unsigned long DoorCycleStart = 0L;
unsigned long DoorCycleEnd = 0L;

String _endl = "\n";
String temp_str;
char temp[50];

//Digital inputs
#define STAT1  12  //Battery charging status
#define STAT2  11  //Battery done charging status
#define SwUP  10  //endswitch UP
#define SwDOWN  20  //endswitch DOWN
#define INTERLCK  20  //Main Door Interlock (not used yet)

//Motor controller pins
#define AIN1 A3
#define AIN2 A2
#define STBY A5

//Up and Down push buttons used to manually force open/close Hen's guillotine door
#define PUSH_BUTTON_UP    0 //INT2
#define PUSH_BUTTON_DOWN  1 //INT3

//Analog input pin to read the Battery level
#define VBATTPIN A1

//Analog input pin to read the ambient luminosity level
#define VLUMPIN A7

//Analog input pin to read the solar panel voltage (better option to measure the ambient luminosity level)
#define SOLPIN A0

// OneWire instance to communicate with the 4 DS18B20 temperature sensors
OneWire oneWire_A(ONE_WIRE_BUS_A);

// Pass our oneWire reference to Dallas Temperature library instance
DallasTemperature sensors_A(&oneWire_A);

//MAC Address of DS18b20 temperature sensor (!unique to every sensor!)
//DeviceAddress DS18b20_4 = { 0x28, 0x5F, 0x93, 0x03, 0x00, 0x00, 0x80, 0xE9 };//Slot1 temp
DeviceAddress DS18b20_4 = { 0x28, 0x9C, 0xD0, 0x00, 0x0C, 0x00, 0x00, 0x37 };//Slot1 temp
DeviceAddress DS18b20_3 = { 0x28, 0xB1, 0x7D, 0x00, 0x0C, 0x00, 0x00, 0x14 };//Slot2 temp
DeviceAddress DS18b20_1 = { 0x28, 0xCB, 0x32, 0xFF, 0x0B, 0x00, 0x00, 0x54 };//Slot3 temp
DeviceAddress DS18b20_2 = { 0x28, 0xD4, 0x68, 0x00, 0x0C, 0x00, 0x00, 0x92 };//Slot4 temp

//12bits (0,06°C) temperature sensors resolution
#define TEMPERATURE_RESOLUTION 12

// Data Packet to Send to TTN
// Bytes 0-1: Average temperature over two bytes
// Bytes 2-3: Slot1 temperature over two bytes
// Bytes 4-5: Slot2 temperature over two bytes
// Bytes 6-7: Slot3 temperature over two bytes
// Bytes 8-9: Slot4 temperature over two bytes
// Bytes 10-11: Luminosity voltage from varistor over two bytes
// Bytes 12-13: Luminosity voltage from solar panel output voltage over two bytes
// Bytes 14-15: Battery voltage over two bytes
// Bytes 16-17: digital inputs reading
static uint8_t loraData[20];
uint8_t DigInputs = 0;
static osjob_t sendjob;

// Pin mapping for Adafruit Feather M0 LoRa
// /!\ By default Adafruit Feather M0's pin 6 and DIO1 are not connected.
// Please ensure they are connected.
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {3, 6, LMIC_UNUSED_PIN},
  .rxtx_rx_active = 0,
  .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
  .spi_freq = 8000000,
};

//pinout of radio module for feather m0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Singleton instance of the radio driver
// Will be used to put the radio module to deep sleep
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//ready to go to sleep?
static bool DoorAtPosition = false;
volatile bool ReadyToSleep = false;
volatile bool JoinedOK = false;

//Interrupt handle if UP push button was pressed
void ButtonUPWake()
{
#ifdef SLEEP
  DoorAtPosition = false;
#endif
  detachInterrupt (digitalPinToInterrupt (PUSH_BUTTON_UP));      // stop LOW interrupt
  ButtonUPPressed = 1;
  ButtonDWNPressed = 0;
  DoorUPStateForced = 1;
  DoorDWNStateForced = 0;
}

//Interrupt handle if DOWN push button was pressed
void ButtonDWNWake()
{
#ifdef SLEEP
  DoorAtPosition = false;
#endif
  detachInterrupt (digitalPinToInterrupt (PUSH_BUTTON_DOWN));      // stop LOW interrupt
  ButtonDWNPressed = 1;
  ButtonUPPressed = 0;
  DoorDWNStateForced = 1;
  DoorUPStateForced = 0;
}

//Set individual bits of the DigInputs Byte
void writeBitmap(bool a, bool b, bool c, bool d, bool e, bool f, bool g, bool h)
{
  // LSB first
  DigInputs = 0;
  DigInputs |= (a & 1) << 7;
  DigInputs |= (b & 1) << 6;
  DigInputs |= (c & 1) << 5;
  DigInputs |= (d & 1) << 4;
  DigInputs |= (e & 1) << 3;
  DigInputs |= (f & 1) << 2;
  DigInputs |= (g & 1) << 1;
  DigInputs |= (h & 1) << 0;
}

void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      JoinedOK = true;
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      ReadyToSleep = true;
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      ReadyToSleep = true;
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      //Loic os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      ReadyToSleep = true;
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      ReadyToSleep = true;
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      ReadyToSleep = true;
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      ReadyToSleep = true;
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      ReadyToSleep = true;
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      ReadyToSleep = true;
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      ReadyToSleep = true;
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while (! Serial);
#endif

  // Initialize pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Show we're awake

  //Digital Inputs
  pinMode(STAT1, INPUT_PULLUP);
  pinMode(STAT2, INPUT_PULLUP);
  pinMode(SwUP, INPUT_PULLUP);
  pinMode(SwDOWN, INPUT_PULLUP);
  pinMode(INTERLCK, INPUT_PULLUP);

  //Digital outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BUZ, OUTPUT);

  //Analog inputs
  analogReadResolution(10);
  pinMode(VLUMPIN, INPUT);
  pinMode(VBATTPIN, INPUT);
  pinMode(SOLPIN, INPUT);

  digitalWrite(STBY, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BUZ, LOW);

  // Initialize push buttons as inputs
  pinMode(PUSH_BUTTON_UP, INPUT_PULLUP);
  pinMode(PUSH_BUTTON_DOWN, INPUT_PULLUP);

  //Enable interrupts
  interrupts();

  // Start up the DS18B20 library
  sensors_A.begin();

  // set the resolution
  sensors_A.setResolution(DS18b20_1, TEMPERATURE_RESOLUTION);
  sensors_A.setResolution(DS18b20_2, TEMPERATURE_RESOLUTION);
  sensors_A.setResolution(DS18b20_3, TEMPERATURE_RESOLUTION);
  sensors_A.setResolution(DS18b20_4, TEMPERATURE_RESOLUTION);

  //Synchronous mode, waits for the reply!
  sensors_A.setWaitForConversion(true);

  //motorized door state machine init
  Door.next(Door_wait);

  // Allow wake up pin to trigger interrupt on low.
  //EIFR = 3;      // cancel any existing falling interrupt (interrupt 2)
  //EIFR = 4;      // cancel any existing falling interrupt (interrupt 3)
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PUSH_BUTTON_UP), ButtonUPWake, LOW);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PUSH_BUTTON_DOWN), ButtonDWNWake, LOW);

  //Initialize radio module
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Disable link-check mode and ADR, because ADR tends to complicate testing.
  //LMIC_setLinkCheckMode(0);
  // Set the data rate to Spreading Factor 7.  This is the fastest supported rate for 125 kHz channels, and it
  // minimizes air time and battery power. Set the transmission power to 14 dBi (25 mW).
  //LMIC_setDrTxpow(DR_SF7,14);
  // in the US, with TTN, it saves join time if we start on subband 1 (channels 8-15). This will
  // get overridden after the join by parameters from the network. If working with other
  // networks or in other regions, this will need to be changed.
  //LMIC_selectSubBand(1);

  // Start job (sending automatically starts OTAA too)
  //send sensors data
  LoraPublish(&sendjob, 3);

  DEBUG_PRINT("Setup() OK!");
}

void loop()
{

  // we call the LMIC's runloop processor. This will cause things to happen based on events and time. One
  // of the things that will happen is callbacks for transmission complete or received messages. We also
  // use this loop to queue periodic data transmissions.  You can put other things here in the `loop()` routine,
  // but beware that LoRaWAN timing is pretty tight, so if you do more than a few milliseconds of work, you
  // will want to call `os_runloop_once()` every so often, to keep the radio running.
  os_runloop_once();

  //if Button UP interrupt has fired
  if (ButtonUPPressed)
  {
    //Door setpoint at 100% (open)
    DoorWantedPos = 100.0;

    //Serial.println(counter);
    ButtonUPPressed = 0;
    ButtonDWNPressed = 0;

    // blink LED to indicate packet sent
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);

    // Allow wake up pin to trigger interrupt on low.
    LowPower.attachInterruptWakeup(digitalPinToInterrupt(PUSH_BUTTON_UP), ButtonUPWake, LOW);
  }

  //if Button DOWN interrupt has fired.
  if (ButtonDWNPressed)
  {
    //Door setpoint at 0% (closed)
    DoorWantedPos = 0.0;

    //Serial.println(counter);
    ButtonDWNPressed = 0;
    ButtonUPPressed = 0;

    // blink LED to indicate packet sent
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);

    // Allow wake up pin to trigger interrupt on low.
    LowPower.attachInterruptWakeup(digitalPinToInterrupt(PUSH_BUTTON_DOWN), ButtonDWNWake, LOW);
  }

  //update Door state engine
  Door.run();


  //Open door at sunrise and close it at sunset, based on an ambient luminosity threshold
  if ((!DoorDWNStateForced) && (measuredSolarlum > LumThreshold_HIGH)) //Door is closed and luminosity > LumThreshold_HIGH% ->open door
  {
    DoorWantedPos = 100.0;
    DoorUPStateForced = 0;
  }
  else if ((!DoorUPStateForced) && (measuredSolarlum < LumThreshold_LOW)) //Door is open and luminosity < LumThreshold_LOW% ->close door
  {
    DoorWantedPos = 0.0;
    DoorDWNStateForced = 0;
  }

  //Open door at sunrise and close it at sunset, based on an ambient luminosity threshold
  if ((!DoorDWNStateForced) && (measuredSolarlum > LumThreshold_HIGH)) //Door is closed and luminosity > LumThreshold_HIGH% ->open door
  {
    DoorWantedPos = 100.0;
    DoorUPStateForced = 0;
  }
  else if ((!DoorUPStateForced) && (measuredSolarlum < LumThreshold_LOW)) //Door is open and luminosity < LumThreshold_LOW% ->close door
  {
    DoorWantedPos = 0.0;
    DoorDWNStateForced = 0;
  }

  if (sleepIterations >= MAX_SLEEP_ITERATIONS)
  {
#ifdef DEBUG
    Serial.begin(115200);
    while (! Serial);
    delay(1000);
    Serial.println("Just woke up!");
#endif
    // Reset the number of sleep iterations.
    sleepIterations = 0;

    //wake radio module and send lora message
    Serial.println("LoRa radio init OK!");

    //send sensors data over LoRaWan on port 3
    LoraPublish(&sendjob, 3);
  }

#ifdef SLEEP
  // Go to sleep if door is at desired position, sensor has successfully joined LoRaWan network and nothing else to do
  if (((int)DoorWantedPos == DoorActualPos) && JoinedOK && ReadyToSleep)
  {
    // Show we're asleep
    digitalWrite(LED_BUILTIN, LOW); 

    //put radio module to deep sleep
    rf95.sleep();

    //put controller to deep sleep
    LowPower.deepSleep(10000);  // Sleep for 10 seconds.

    //We are awake
#ifdef DEBUG
    digitalWrite(LED_BUILTIN, HIGH); // Show we're not asleep anymore
#endif
    sleepIterations += 1;
  }
#endif
}

void LoraPublish(osjob_t* j, unsigned char port)
{
  ReadyToSleep = false;

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    //port 3 is used to send sensors data at regular intervals
    if (port == 3)
    {
      sensors_A.requestTemperatures();
      float S1 = sensors_A.getTempC(DS18b20_1);
      float S2 = sensors_A.getTempC(DS18b20_2);
      float S3 = sensors_A.getTempC(DS18b20_3);
      float S4 = sensors_A.getTempC(DS18b20_4);
      float tm = (S1 + S2 + S3 + S4) / 4.0;

      // scale to -1 to 1 range
      S1 /= 100.0;
      S2 /= 100.0;
      S3 /= 100.0;
      S4 /= 100.0;
      tm /= 100.0;

      // float -> int
      // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)
      uint16_t payloadS1 = LMIC_f2sflt16(S1);
      uint16_t payloadS2 = LMIC_f2sflt16(S2);
      uint16_t payloadS3 = LMIC_f2sflt16(S3);
      uint16_t payloadS4 = LMIC_f2sflt16(S4);
      uint16_t payloadtm = LMIC_f2sflt16(tm);

      //Read luminosity level
      measuredvlum = analogRead(VLUMPIN);
      measuredvlum /= 1024; // convert to 0-1 range
      // float -> int
      // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)
      uint16_t payloadlum = LMIC_f2sflt16(measuredvlum);

      //Read solar panel luminosity level
      measuredSolarlum = analogRead(SOLPIN);
      measuredSolarlum /= 1024; // convert to 0-1 range
      // float -> int
      // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)
      uint16_t payloadsolarlum = LMIC_f2sflt16(measuredSolarlum);

      //Read battery level
      float measuredbatt = analogRead(VBATTPIN);
      measuredbatt *= 6.6;  // 100% = 3.3*2V because of voltage divider
      measuredbatt /= 1024; // convert to %
      measuredbatt /= 100.0; // scale to 0-1 range
      // float -> int
      // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)
      uint16_t payloadbatt = LMIC_f2sflt16(measuredbatt);

      //Read the Digital inputs states and record them into a Byte to be sent in the message payload
      writeBitmap(false, digitalRead(INTERLCK), digitalRead(SwDOWN), digitalRead(SwUP), digitalRead(STAT2), digitalRead(STAT1), ButtonUPPressed, ButtonDWNPressed);

#ifdef DEBUG
      String msg = F("VBat: ");
      msg += measuredbatt;
      msg += F("V\t");
      msg += F("Lum: ");
      msg += measuredvlum;
      msg += F("%\t");
      msg += F("SoLum: ");
      msg += measuredSolarlum;
      msg += F("%\t");
      msg += F("Avg Temp: ");
      msg += tm;
      msg += F("°C\t");
      msg += F("S1 Temp: ");
      msg += S1;
      msg += F("°C\t");
      msg += F("S2 Temp: ");
      msg += S2;
      msg += F("°C\t");
      msg += F("S3 Temp: ");
      msg += S3;
      msg += F("°C\t");
      msg += F("S4 Temp: ");
      msg += S4;
      msg += F("°C\t");
      msg += F("DigInputs: ");
      msg += DigInputs;

      DEBUG_PRINT(msg);
#endif

      // encode int as bytes
      loraData[0] = lowByte(payloadtm);
      loraData[1] = highByte(payloadtm);

      loraData[2] = lowByte(payloadS1);
      loraData[3] = highByte(payloadS1);

      loraData[4] = lowByte(payloadS2);
      loraData[5] = highByte(payloadS2);

      loraData[6] = lowByte(payloadS3);
      loraData[7] = highByte(payloadS3);

      loraData[8] = lowByte(payloadS4);
      loraData[9] = highByte(payloadS4);

      loraData[10] = lowByte(payloadlum);
      loraData[11] = highByte(payloadlum);

      loraData[12] = lowByte(payloadsolarlum);
      loraData[13] = highByte(payloadsolarlum);

      loraData[14] = lowByte(payloadbatt);
      loraData[15] = highByte(payloadbatt);

      loraData[16] = highByte(0);
      loraData[17] = lowByte(DigInputs);
    }
    else if (port == 4) //port 4 is used to send door opening/closing progress at short intervals only when door is moving
    {
      //Read the Digital inputs states and record them into a Byte to be sent in the message payload
      writeBitmap(false, digitalRead(INTERLCK), digitalRead(SwDOWN), digitalRead(SwUP), digitalRead(STAT2), digitalRead(STAT1), ButtonUPPressed, ButtonDWNPressed);

#ifdef DEBUG
      String msg = F("Door wanted pos: ");
      msg += DoorWantedPos;
      msg += F("%");
      msg += F(" - Door actual pos: ");
      msg += DoorActualPos;
      msg += F("%\t");
      msg += F("DigInputs: ");
      msg += DigInputs;

      DEBUG_PRINT(msg);
#endif

      //convert float to int
      int iDoorWantedPos = round(DoorWantedPos);

      // encode int as bytes
      loraData[0] = highByte(iDoorWantedPos);
      loraData[1] = lowByte(iDoorWantedPos);

      loraData[2] = highByte(DoorActualPos);
      loraData[3] = lowByte(DoorActualPos);

      loraData[4] = highByte(0);
      loraData[5] = lowByte(DigInputs);
    }

    DEBUG_PRINT("Sending LoRa Data...");

    // prepare upstream data transmission at the next possible time.
    // transmit on "port" (the first parameter); you can use any value from 1 to 223 (others are reserved).
    // don't request an ack (the last parameter, if not zero, requests an ack from the network).
    // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
    LMIC_setTxData2(port, loraData, sizeof(loraData) - 1, 0);

    // blink LED to indicate packet sent
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

//open or close Door
void moveDoor(bool _direction)
{
  if (_direction) //open
  {
    DoorMoving = true;
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    if ((digitalRead(SwUP) != InContact)) //open door if top magnet is not in contact
      digitalWrite(STBY, HIGH);
    else
      digitalWrite(STBY, LOW);

  }
  else
  {
    DoorMoving = true;
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH);
    if ((digitalRead(SwDOWN) != InContact)) //close door if bottom magnet is not in contact
      digitalWrite(STBY, HIGH);
    else
      digitalWrite(STBY, LOW);
  }
}

void stopDoor()
{
  digitalWrite(STBY, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  DoorMoving = false;
}
////////////////////////Door state machine///////////////////////////////////////

void Door_wait()
{
  /*  String UPSW = "UP_SW: "; UPSW +=digitalRead(SwUP);
    DEBUG_PRINT(UPSW);
    String DWNSW = "DWN_SW: "; DWNSW +=digitalRead(SwDOWN);
    DEBUG_PRINT(DWNSW);
  */
  if (Door.isFirstRun())
  {
    String msg = "Actual: "; msg += DoorActualPos; msg += " - Wanted: "; msg += DoorWantedPos;
    DEBUG_PRINT(msg);
  }
  //DoorWantedPos = round(DoorWantedPos);
  if (DoorWantedPos < 0) DoorWantedPos = 0;
  if (DoorWantedPos > 100) DoorWantedPos = 100;
  if (DoorActualPos < 0) DoorActualPos = 0;
  if (DoorActualPos > 100) DoorActualPos = 100;

  //Serial<<F("round(DoorActualPos): ")<<round(DoorActualPos)<<F(" ")<<DoorWantedPos<<_endl;

  if (digitalRead(SwUP) == InContact) DoorActualPos = 100.0;
  if (digitalRead(SwDOWN) == InContact) DoorActualPos = 0.0;

  if (DoorActualPos == DoorWantedPos)
  {
    if (DoorMoving)
    {
      String msg = "Actual: "; msg += DoorActualPos; msg += " - Wanted: "; msg += DoorWantedPos;
      DEBUG_PRINT(msg);
    }
    stopDoor();
    //DoorCycleEnd = millis();
  }
  if (DoorActualPos < DoorWantedPos) Door.next(Door_moveOpen);
  if (DoorActualPos > DoorWantedPos) Door.next(Door_moveClose);

}

void Door_moveOpen()
{
  if (Door.isFirstRun())
  {
    //DoorCycleStart = millis();
    String msg = "Actual: "; msg += DoorActualPos; msg += " - Wanted: "; msg += DoorWantedPos;
    DEBUG_PRINT(msg);
  }

  if (digitalRead(SwUP) == InContact) DoorActualPos = 100.0;
  if (digitalRead(SwDOWN) == InContact) DoorActualPos = 0.0;
  if (DoorActualPos == DoorWantedPos)
    Door.next(Door_wait);

  moveDoor(DoorOPEN);
  if (Door.elapsed(DoorTimeConstant / 10))
  {
    DoorActualPos += 10;
#ifdef DOOR_PROGRESS
    LoraPublish(&sendjob, 4);
#endif
#ifdef DOOR_BUZZ
    tone(BUZ, Frequ, Duration);
#endif
    String msg = "Actual: "; msg += DoorActualPos; msg += " - Wanted: "; msg += DoorWantedPos;
    DEBUG_PRINT(msg);
    Door.next(Door_wait);
  }
}

void Door_moveClose()
{
  if (Door.isFirstRun())
  {
    //DoorCycleStart = millis();
    String msg = "Actual: "; msg += DoorActualPos; msg += " - Wanted: "; msg += DoorWantedPos;
    DEBUG_PRINT(msg);
  }

  if (digitalRead(SwUP) == InContact) DoorActualPos = 100.0;
  if (digitalRead(SwDOWN) == InContact) DoorActualPos = 0.0;
  if (DoorActualPos == DoorWantedPos)
    Door.next(Door_wait);

  moveDoor(DoorCLOSE);
  if (Door.elapsed(DoorTimeConstant / 10))
  {
    DoorActualPos -= 10;
#ifdef DOOR_PROGRESS
    LoraPublish(&sendjob, 4);
#endif
#ifdef DOOR_BUZZ
    tone(BUZ, Frequ, Duration);
#endif
    String msg = "Actual: "; msg += DoorActualPos; msg += " - Wanted: "; msg += DoorWantedPos;
    DEBUG_PRINT(msg);
    Door.next(Door_wait);
  }
}
