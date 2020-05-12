/* HenHouse (c) Loic74 <loic74650@gmail.com> 2020
  HenHouse solar-powered intelligent controller:
  UP/DOWN interface buttons to control Guillotine door opening/closing
  Door has endswitches to stop opening/closing as well as a timeout if endswitches were not reached in time
  Measures temperature in 4 egg-slots + an average of the 4 sensors
  Measures battery voltage as well as charging state (charging, done)
  Goes to DeepSleep until UP/DOWN buttons are pressed or until watchdog wakes it up (every 5 minutes) to broadcast measured inputs data over LoRaWan

  Visit your thethingsnetwork.org device console
  to create an account, and obtain the session keys below which are unique to every board.

  You should then create a file called "arduino_secrets.h", save it in the project folder, and into which you will paste those unique keys in the following form:

  -------------------------------------------------------------------------------------------
  // Network Session Key (MSB)
  uint8_t NwkSkey[16] = { 0x4C, 0xD5, 0xA5, 0x70, 0xFE, 0x7B, 0xC5, 0xDE, 0x52, XXXXXXXXXXXXXXXXXXXXXXXXX };

  // Application Session Key (MSB)
  uint8_t AppSkey[16] = { 0x88, 0xB5, 0x4C, 0x7B, 0xD2, 0x12, 0x8B, 0xE7, 0x6A, XXXXXXXXXXXXXXXXXXXXXXXXX };

  // Device Address (MSB)
  uint8_t DevAddr[4] = { 0x26, XXXXXXXXXXXXXXXXX };

  --------------------------------------------------------------------------------------------

***Dependencies and respective revisions used to compile this project***
  https://github.com/PaulStoffregen/OneWire (rev 2.3.4)
  https://github.com/milesburton/Arduino-Temperature-Control-Library (rev 3.7.2)
  https://github.com/bricofoy/yasm (rev 0.9.2)
  https://github.com/adafruit/TinyLoRa/ (rev 1.0.4)

  TODO:
  Add timeout on door opening closing in case endswitches fail. Report error over MQTT if timeout reached
  Add reset every day
  When manually opening door, let it as is for a certain amount of time before letting it automatically go to its position per luminosity
*/

/*
  //TTN decoder function:

  function Decoder(bytes, port) {
  var decoded = {};

  if(port == 3)
  {
  // Decode bytes to int
    var tmInt = (bytes[0] << 8) | bytes[1];
    var S1Int = (bytes[2] << 8) | bytes[3];
    var S2Int = (bytes[4] << 8) | bytes[5];
    var S3Int = (bytes[6] << 8) | bytes[7];
    var S4Int = (bytes[8] << 8) | bytes[9];
    var LumInt = (bytes[10] << 8) | bytes[11];
    var BattInt = (bytes[12] << 8) | bytes[13];
    var DigInt = (bytes[14] << 8) | bytes[15];

    // Decode int to float
    decoded.Mtemp = tmInt / 100;
    decoded.S1temp = S1Int / 100;
    decoded.S2temp = S2Int / 100;
    decoded.S3temp = S3Int / 100;
    decoded.S4temp = S4Int / 100;
    decoded.lum = LumInt / 100;
    decoded.bat = BattInt / 100;

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
  }*/
/************************** Configuration ***********************************/

#define SLEEP             ->comment this line to prevent µc from sleeping
//#define DOOR_PROGRESS   ->comment this line to prevent code from broadcasting door % position while opening/closing (saves Lora airtime)

//#define DEBUG           ->comment this line to prevent code from writing debug messages to serial port (must be commented when SLEEP is uncommented)
#include "DebugUtils.h"

// Data logging configuration.
#define LOGGING_FREQ_SECONDS   60       // Seconds to wait before a new sensor reading is logged.


#include <TinyLoRa.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include "OneWire.h"
#include <DallasTemperature.h>
#include <yasm.h>
#include <Streaming.h>
#include "arduino_secrets.h" //this file should be placed in same folder as the sketch and contains your TTN session keys, see below for more details

// Firmware revision
String Firmw = "0.0.1";


#define MAX_SLEEP_ITERATIONS   LOGGING_FREQ_SECONDS / 8  // Number of times to sleep (for 8 seconds) before
// a sensor reading is taken and sent to the server.
// Don't change this unless you also change the
// watchdog timer configuration.

#define LumThreshold_LOW  5                 //Luminosity threshold to actuate door in the evenings
#define LumThreshold_HIGH 30                //Luminosity threshold to actuate door in the mornings

volatile bool watchdogActivated = false;
volatile bool ButtonUPPressed = false;
volatile bool ButtonDWNPressed = false;
volatile bool DoorMoving = false;
volatile bool DoorDWNStateForced = false;
volatile bool DoorUPStateForced = false;
volatile float measuredvlum = LumThreshold_HIGH;

volatile int sleepIterations = 0;

// Data wire is connected to input digital pin A4 of the Adafruit Feather 32u4 LoRa
#define ONE_WIRE_BUS_A A4

//Motorized door state machine
volatile YASM Door;

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
#define SwDOWN  9  //endswitch DOWN
#define INTERLCK  9  //Main Door Interlock (not used yet)

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
#define VLUMPIN A0

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
// Bytes 10-11: Luminosity voltage over two bytes
// Bytes 12-13: Battery voltage over two bytes
// Bytes 14-15: digital inputs reading
unsigned char loraData[16];
uint8_t DigInputs = 0;
int16_t lumInt = 0;
int16_t battInt = 0;
int16_t tmInt, S1Int, S2Int, S3Int, S4Int;

// Pinout for Adafruit Feather 32u4 LoRa
TinyLoRa lora = TinyLoRa(7, 8);

//Interrupt handle if UP push button was pressed
void ButtonUPWake()
{
  #ifdef SLEEP
    sleep_disable ();         // first thing after waking from sleep:
  #endif
  detachInterrupt (digitalPinToInterrupt (PUSH_BUTTON_UP));      // stop LOW interrupt
  //wdt_disable();  // disable watchdog
  ButtonUPPressed = 1;
  ButtonDWNPressed = 0;
  DoorUPStateForced = 1;
  DoorDWNStateForced = 0;
}

//Interrupt handle if DOWN push button was pressed
void ButtonDWNWake()
{
  #ifdef SLEEP
    sleep_disable ();         // first thing after waking from sleep:
  #endif
  detachInterrupt (digitalPinToInterrupt (PUSH_BUTTON_DOWN));      // stop LOW interrupt
  //wdt_disable();  // disable watchdog
  ButtonDWNPressed = 1;
  ButtonUPPressed = 0;
  DoorDWNStateForced = 1;
  DoorUPStateForced = 0;
}


// Define watchdog timer interrupt.
ISR(WDT_vect)
{
  // Set the watchdog activated flag.
  // Note that you shouldn't do much work inside an interrupt handler.
  #ifdef SLEEP
    sleep_disable ();         // first thing after waking from sleep:
  #endif
  watchdogActivated = true;
}

void SetupWDog()
{
  // Setup the watchdog timer to run an interrupt which
  // wakes the Arduino from sleep every 8 seconds.

  // Note that the default behavior of resetting the Arduino
  // with the watchdog will be disabled.

  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts();

  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1 << WDRF);

  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1 << WDP0) | (1 << WDP3);

  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1 << WDIE);

  // Enable interrupts again.
  interrupts();
}

#ifdef SLEEP
// Put the Arduino to sleep.
void sleep()
{
  // Set sleep to full power down.  Only external interrupts or
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Turn off the ADC while asleep.
  power_adc_disable();

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point.

  // When awake, disable sleep mode and turn on all devices.
  //sleep_disable();
  power_all_enable();
}
#endif

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

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while (! Serial);
#endif

  // Initialize pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);

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

  digitalWrite(STBY, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  // Initialize push buttons as inputs
  pinMode(PUSH_BUTTON_UP, INPUT_PULLUP);
  pinMode(PUSH_BUTTON_DOWN, INPUT_PULLUP);

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
  EIFR = 3;      // cancel any existing falling interrupt (interrupt 2)
  EIFR = 4;      // cancel any existing falling interrupt (interrupt 3)
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_UP), ButtonUPWake, LOW);
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_DOWN), ButtonDWNWake, LOW);

  SetupWDog();

  // Initialize LoRa
  // Make sure Region #define is correct in TinyLora.h file
  DEBUG_PRINT("Starting LoRa...");
  // define multi-channel sending
  lora.setChannel(MULTI);
  // set datarate
  lora.setDatarate(SF7BW125);//fast and low power because we are close to gateway
  if (!lora.begin())
  {
    DEBUG_PRINT("Failed. Check your radio");
    while (true);
  }
  DEBUG_PRINT("OK");
}

void loop()
{
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
    EIFR = 3;      // cancel any existing falling interrupt (interrupt 2)
    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_UP), ButtonUPWake, LOW);
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
    EIFR = 4;      // cancel any existing falling interrupt (interrupt 3)
    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_DOWN), ButtonDWNWake, LOW);
  }

  //update Door state engine
  Door.run();

  //if watchdog interrupt has fired
  if (watchdogActivated)
  {
    watchdogActivated = false;
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
    if (sleepIterations >= MAX_SLEEP_ITERATIONS)
    {
      // Reset the number of sleep iterations.
      sleepIterations = 0;

      //send sensors data
      LoraPublish(3);
    }
  }

  //Open door at sunrise and close it at sunset, based on a 30% ambient luminosity threshold
  if ((!DoorDWNStateForced) && (measuredvlum > LumThreshold_HIGH)) //Door is closed and luminosity > LumThreshold_HIGH% ->open door
  {
    DoorWantedPos = 100.0;
    DoorUPStateForced = 0;
  }
  else 
  if((!DoorUPStateForced) && (measuredvlum < LumThreshold_LOW)) //Door is open and luminosity < LumThreshold_LOW% ->close door
  {
    DoorWantedPos = 0.0;
    DoorDWNStateForced = 0;
  }

#ifdef SLEEP
  // Go to sleep if door is at desired position and nothing else to do
  if ((int)DoorWantedPos == DoorActualPos)
    sleep();
#endif
}

void LoraPublish(unsigned char port)
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

    //Read luminosity level
    measuredvlum = analogRead(VLUMPIN);
    measuredvlum *= 100.0;  // 100%
    measuredvlum /= 1024; // convert to %

    //Read battery level
    float measuredbatt = analogRead(VBATTPIN);
    measuredbatt *= 6.6;  // 100% = 3.3*2V because of voltage divider
    measuredbatt /= 1024; // convert to %

    // encode float as int
    lumInt = round(measuredvlum * 100);
    battInt = round(measuredbatt * 100);
    tmInt = round(tm * 100);
    S1Int = round(S1 * 100);
    S2Int = round(S2 * 100);
    S3Int = round(S3 * 100);
    S4Int = round(S4 * 100);

    //Read the Digital inputs states and record them into a Byte to be sent in the message payload
    writeBitmap(false, digitalRead(INTERLCK), digitalRead(SwDOWN), digitalRead(SwUP), digitalRead(STAT2), digitalRead(STAT1), ButtonUPPressed, ButtonDWNPressed);

#ifdef DEBUG
    String msg = F("VBat: ");
    msg += measuredbatt;
    msg += F("V\t");
    msg += F("Lum: ");
    msg += measuredvlum;
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
    loraData[0] = highByte(tmInt);
    loraData[1] = lowByte(tmInt);

    loraData[2] = highByte(S1Int);
    loraData[3] = lowByte(S1Int);

    loraData[4] = highByte(S2Int);
    loraData[5] = lowByte(S2Int);

    loraData[6] = highByte(S3Int);
    loraData[7] = lowByte(S3Int);

    loraData[8] = highByte(S4Int);
    loraData[9] = lowByte(S4Int);

    loraData[10] = highByte(lumInt);
    loraData[11] = lowByte(lumInt);

    loraData[12] = highByte(battInt);
    loraData[13] = lowByte(battInt);

    loraData[14] = highByte(0);
    loraData[15] = lowByte(DigInputs);
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

  lora.sendData(loraData, sizeof(loraData), port, lora.frameCounter);

#ifdef DEBUG
  String msg = F("Frame Counter: ");
  msg += lora.frameCounter;
  DEBUG_PRINT(msg);
#endif

  lora.frameCounter++;

  // blink LED to indicate packet sent
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
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
  if(DoorActualPos == DoorWantedPos)
  Door.next(Door_wait);
  
  moveDoor(DoorOPEN);
  if(Door.elapsed(DoorTimeConstant / 10))
  {
    DoorActualPos += 10;
#ifdef DOOR_PROGRESS
    LoraPublish(4);
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
  if(DoorActualPos == DoorWantedPos)
  Door.next(Door_wait);
  
  moveDoor(DoorCLOSE);
  if (Door.elapsed(DoorTimeConstant / 10))
  {
    DoorActualPos -= 10;
#ifdef DOOR_PROGRESS
    LoraPublish(4);
#endif
    String msg = "Actual: "; msg += DoorActualPos; msg += " - Wanted: "; msg += DoorWantedPos;
    DEBUG_PRINT(msg);
    Door.next(Door_wait);
  }
}
