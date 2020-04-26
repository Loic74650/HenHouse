/* HenHouse (c) Loic74 <loic74650@gmail.com> 2020
HenHouse solar-powered intelligent controller:
UP/DOWN interface buttons to control Guillotine door opening/closing
Door has endswitches to stop opening/closing as well as a timeout if endswitches were not reached in time 
Measures temperature in 4 egg-slots + in main area
Measures battery voltage as well as charging state (charging, done)
Reports all measured inputs over LoRaWan every minute or when an interface button is activated
Goes to DeepSleep until UP/DOWN buttons are pressed + wakes up every minute to broadcast data over LoRaWan measured inputs

Plug the Adafruit solar charger STAT1 digital output (Battery charging status) to digital input pin 2 of Feather board
Plug the Adafruit solar charger STAT2 digital output (Battery done charging status) to digital input pin 3 of Feather board
Plug guillotine door endswitch UP to digital input pin 5 and GND of Feather board
Plug guillotine door endswitch DOWN to digital input pin 6 and GND of Feather board
Plug Large side door interlock switch to digital input pin 9 and GND of Feather board
Plug door UP button between pin 0 and GND
Plug door DOWN button between pin 1 and GND


***Dependencies and respective revisions used to compile this project***
https://github.com/PaulStoffregen/OneWire (rev 2.3.4)
https://github.com/milesburton/Arduino-Temperature-Control-Library (rev 3.7.2)
https://github.com/bricofoy/yasm (rev 0.9.2)
https://github.com/adafruit/TinyLoRa/ (rev 1.0.4)

*/
/*
 TODO: 
 read digital inputs from motor up/down endswitches and broadcast them
 Add state machine for when door is opening/closing
 Add reading/broadcasting of multiple temp probes
 USe port 3 for TTN decoding
 Add timeout on door opening closing in case endswitches fail. Report error over MQTT if timeout reached
 Add downward commands to open/close doors
 Add reading luminosity and open/close door accordingly. Add hysteresis
 Check what happens when pressing buttons while system already awake (deboucne class event?)
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
    var BattInt = (bytes[10] << 8) | bytes[11];
    var DigInt = (bytes[12] << 8) | bytes[13];
         
    // Decode int to float
    decoded.Mtemp = tmInt / 100;
    decoded.S1temp = S1Int / 100;
    decoded.S2temp = S2Int / 100;
    decoded.S3temp = S3Int / 100;
    decoded.S4temp = S4Int / 100;
    decoded.batt = BattInt / 100;

    // Decode digital inputs
    decoded.ButtonDown = (DigInt & 1) === 0?0:1;
    decoded.ButtonUp = (DigInt & 2) === 0?0:1;
    decoded.Charging = (DigInt & 3) === 0?0:1;
    decoded.DoneCharging = (DigInt & 4) === 0?0:1;
    decoded.DoorSwUp = (DigInt & 5) === 0?1:0;
    decoded.DoorSwDn = (DigInt & 6) === 0?1:0;
    decoded.DoorIntck = (DigInt & 7) === 0?1:0;
    return decoded;
  }
}*/
/************************** Configuration ***********************************/
#include <TinyLoRa.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include "OneWire.h"
#include <DallasTemperature.h>
#include <yasm.h>
#include <Streaming.h>

// Firmware revision
String Firmw = "0.0.1";

// Data logging configuration.
#define LOGGING_FREQ_SECONDS   60       // Seconds to wait before a new sensor reading is logged.

#define MAX_SLEEP_ITERATIONS   LOGGING_FREQ_SECONDS / 8  // Number of times to sleep (for 8 seconds) before
                                                         // a sensor reading is taken and sent to the server.
                                                         // Don't change this unless you also change the 
                                                         // watchdog timer configuration.

volatile bool watchdogActivated = false;
volatile bool ButtonUPPressed = false;
volatile bool ButtonDWNPressed = false;

int sleepIterations = 0;
const int debounceTime = 20;  // debounce in milliseconds

// Data wire is connected to input digital pin A4 of the Adafruit Feather 32u4 LoRa
#define ONE_WIRE_BUS_A A4

// Analog port to measure Luminosity
#define LumPort A0

//Motorized door state machine
YASM Door;

//Door position
double DoorWantedPos = 100.0;
int DoorActualPos = 100;

//relay pins to actuate DC motor CW or CCW
#define PIN_R0 0 
#define PIN_R1 1 

//Door movements
#define DoorOPEN     1
#define DoorCLOSE    0
unsigned long DoorTimeConstant = 30000L; //30 sec

String _endl = "\n";
String temp_str;
char temp[50];

//Digital inputs
#define STAT1  12  //Battery charging status
#define STAT2  11  //Battery done charging status
#define SwUP  10  //endswitch UP
#define SwDOWN  9  //endswitch DOWN
#define INTERLCK  9  //Main Door Interlock

//Up and Down push buttons used to open/close Hen's guillotine door
#define PUSH_BUTTON_UP    0 //INT2
#define PUSH_BUTTON_DOWN  1 //INT3

//Analog input pin to read the Battery level
#define VBATPIN A0

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire_A(ONE_WIRE_BUS_A);

// Pass our oneWire reference to Dallas Temperature library instance 
DallasTemperature sensors_A(&oneWire_A);

//MAC Address of DS18b20 temperature sensor (!unique to every sensor!)
DeviceAddress DS18b20_0 = { 0x28, 0xD4, 0x68, 0x00, 0x0C, 0x00, 0x00, 0x92 };//Main temp
DeviceAddress DS18b20_4 = { 0x28, 0x9C, 0xD0, 0x00, 0x0C, 0x00, 0x00, 0x37 };//Slot1 temp
DeviceAddress DS18b20_3 = { 0x28, 0xB1, 0x7D, 0x00, 0x0C, 0x00, 0x00, 0x14 };//Slot2 temp
DeviceAddress DS18b20_1 = { 0x28, 0xCB, 0x32, 0xFF, 0x0B, 0x00, 0x00, 0x54 };//Slot3 temp
DeviceAddress DS18b20_2 = { 0x28, 0xD4, 0x68, 0x00, 0x0C, 0x00, 0x00, 0x92 };//Slot4 temp

//12bits (0,06°C) temperature sensor resolution
#define TEMPERATURE_RESOLUTION 12

// Visit your thethingsnetwork.org device console
// to create an account, and obtain the session keys below.


// Network Session Key (MSB)
uint8_t NwkSkey[16] = { 0x4C, 0xD5, 0xA5, 0x70, 0xFE, 0x7B, 0xC5, 0xDE, 0x52, 0x61, 0xE8, 0x3A, 0x2C, 0x1D, 0x46, 0x4A };

// Application Session Key (MSB)
uint8_t AppSkey[16] = { 0x88, 0xB5, 0x4C, 0x7B, 0xD2, 0x12, 0x8B, 0xE7, 0x6A, 0x75, 0x15, 0x16, 0x70, 0xE8, 0xB7, 0x45 };

// Device Address (MSB)
uint8_t DevAddr[4] = { 0x26, 0x01, 0x15, 0x94 };

// Data Packet to Send to TTN
// Bytes 0-1: Main temperature over two bytes
// Bytes 2-3: Slot1 temperature over two bytes
// Bytes 4-5: Slot2 temperature over two bytes
// Bytes 6-7: Slot3 temperature over two bytes
// Bytes 8-9: Slot4 temperature over two bytes
// Bytes 10-11: Battery voltage over two bytes
// Bytes 12-13: digital inputs reading
unsigned char loraData[14];
uint8_t DigInputs = 0;
int16_t batteryInt = 0;
int16_t tmInt, S1Int, S2Int, S3Int, S4Int;

// Pinout for Adafruit Feather 32u4 LoRa
TinyLoRa lora = TinyLoRa(7, 8);

// Define watchdog timer interrupt.
ISR(WDT_vect)
{
  // Set the watchdog activated flag.
  // Note that you shouldn't do much work inside an interrupt handler.
  sleep_disable ();         // first thing after waking from sleep:
  watchdogActivated = true;
}

//Interrupt handle if UP push button was pressed
void ButtonUPWake()
{
   sleep_disable ();         // first thing after waking from sleep:
   detachInterrupt (digitalPinToInterrupt (PUSH_BUTTON_UP));      // stop LOW interrupt
   //wdt_disable();  // disable watchdog
   ButtonUPPressed = 1;
   ButtonDWNPressed = 0;
}

//Interrupt handle if DOWN push button was pressed
void ButtonDWNWake()
{
   sleep_disable ();         // first thing after waking from sleep:
   detachInterrupt (digitalPinToInterrupt (PUSH_BUTTON_DOWN));      // stop LOW interrupt 
   //wdt_disable();  // disable watchdog
   ButtonDWNPressed = 1;
   ButtonUPPressed = 0;
}

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

void deBounceUP ()
{
  unsigned long now = millis ();
  do
  {
    // on bounce, reset time-out
    if (digitalRead (PUSH_BUTTON_UP) == LOW)
      now = millis ();
  } 
  while (digitalRead (PUSH_BUTTON_UP) == LOW ||
    (millis () - now) <= debounceTime);

}  // end of deBounce

void setup()
{
  delay(2000);
  Serial.begin(9600);
  
  //uncomment this in debug mode
  //while (! Serial);
 
  // Initialize pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);

  //Digital Inputs
  pinMode(STAT1, INPUT_PULLUP);
  pinMode(STAT2, INPUT_PULLUP);
  pinMode(SwUP, INPUT_PULLUP);
  pinMode(SwDOWN, INPUT_PULLUP);
  pinMode(INTERLCK, INPUT_PULLUP);
    
  // Initialize push buttons as inputs
  pinMode(PUSH_BUTTON_UP, INPUT_PULLUP);
  pinMode(PUSH_BUTTON_DOWN, INPUT_PULLUP);

  // Start up the 18DS20 library
  sensors_A.begin();
       
  // set the resolution
  sensors_A.setResolution(DS18b20_0, TEMPERATURE_RESOLUTION);

  //Synchronous mode
  sensors_A.setWaitForConversion(true);

  //motorized door state machine init
//  Door.next(Door_wait);

  // Allow wake up pin to trigger interrupt on low.
  EIFR = 3;      // cancel any existing falling interrupt (interrupt 2)
  EIFR = 4;      // cancel any existing falling interrupt (interrupt 3)
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_UP), ButtonUPWake, LOW);
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_DOWN), ButtonDWNWake, LOW);
  
  // Setup the watchdog timer to run an interrupt which
  // wakes the Arduino from sleep every 8 seconds.
  
  // Note that the default behavior of resetting the Arduino
  // with the watchdog will be disabled.
  
  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts();
  
  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);
  
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP3);
  
  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);
  
  // Enable interrupts again.
  interrupts();
  
    // Initialize LoRa
  // Make sure Region #define is correct in TinyLora.h file
  Serial.print("Starting LoRa...");
  // define multi-channel sending
  lora.setChannel(MULTI);
  // set datarate
  lora.setDatarate(SF7BW125);//fast and low power because we are close to gateway
  if(!lora.begin())
  {
    Serial.println("Failed");
    Serial.println("Check your radio");
    while(true);
  }
  Serial.println("OK");
}

void loop()
{
  //Door.run();
  
  // Don't do anything unless the watchdog timer interrupt has fired.
   if(ButtonUPPressed)
    {
      // Do something here
      // Example: Read sensor, data logging, data transmission.
      // blink LED to indicate packet sent
      //deBounce ();
      
      LoraPublish();
      
      //Serial.println(counter);
      ButtonUPPressed = 0;
      
      // Allow wake up pin to trigger interrupt on low.
      EIFR = 3;      // cancel any existing falling interrupt (interrupt 2)
      attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_UP), ButtonUPWake, LOW);  
    }
    if(ButtonDWNPressed)
    {
      // Do something here
      // Example: Read sensor, data logging, data transmission.
      // blink LED to indicate packet sent
      //deBounce ();
      
      LoraPublish();
           
      //Serial.println(counter);
      ButtonDWNPressed = 0;
      
      // Allow wake up pin to trigger interrupt on low.
      EIFR = 4;      // cancel any existing falling interrupt (interrupt 3)
      attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_DOWN), ButtonDWNWake, LOW);  
    }
      
  if(watchdogActivated)
  {
     watchdogActivated = false;
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
    if (sleepIterations >= MAX_SLEEP_ITERATIONS) 
    {
      // Reset the number of sleep iterations.
      sleepIterations = 0;

      LoraPublish();
    }
  }

    // Go to sleep!
    sleep();
}

void LoraPublish()
{
      sensors_A.requestTemperatures();
      float tm = sensors_A.getTempC(DS18b20_0);
      float S1 = sensors_A.getTempC(DS18b20_1);
      float S2 = sensors_A.getTempC(DS18b20_2);
      float S3 = sensors_A.getTempC(DS18b20_3);
      float S4 = sensors_A.getTempC(DS18b20_4); 
      
      //Read battery level
      float measuredvbat = analogRead(VBATPIN);
      measuredvbat *= 2;    // we divided by 2 with the resistors bridge, so multiply back
      measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
      measuredvbat /= 1024; // convert to voltage
      
      // encode float as int
      batteryInt = round(measuredvbat * 100);
      tmInt = round(tm * 100);
      S1Int = round(S1 * 100);
      S2Int = round(S2 * 100);
      S3Int = round(S3 * 100);
      S4Int = round(S4 * 100);
              
      //Read the Digital inputs states and record them into a Byte to be sent in the message payload (not used in this sketch) 
      writeBitmap(false, digitalRead(INTERLCK), digitalRead(SwDOWN), digitalRead(SwUP), digitalRead(STAT2), digitalRead(STAT1), ButtonUPPressed, ButtonDWNPressed);
       
      Serial.print("VBat: " ); 
      Serial.print(measuredvbat);
      Serial.print("V\t");
      Serial.print("Main Temp: ");
      Serial.print(tm);
      Serial.print("°C\t");
      Serial.print("S1 Temp: ");
      Serial.print(S1);
      Serial.print("°C\t");
      Serial.print("S2 Temp: ");
      Serial.print(S2);
      Serial.print("°C\t");
      Serial.print("S3 Temp: ");
      Serial.print(S3);
      Serial.print("°C\t");
      Serial.print("S4 Temp: ");
      Serial.print(S4);
      Serial.print("°C\t");
      Serial.print("DigInputs: ");
      Serial.println(DigInputs);
    
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
      
      loraData[10] = highByte(batteryInt);
      loraData[11] = lowByte(batteryInt);
    
      loraData[12] = highByte(0);
      loraData[13] = lowByte(DigInputs);
    
      Serial.println("Sending LoRa Data...");
      lora.sendData(loraData, sizeof(loraData), 3, lora.frameCounter);
      Serial.print("Frame Counter: ");Serial.println(lora.frameCounter);
      lora.frameCounter++;
    
      // blink LED to indicate packet sent
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);  
}

//open or close Door
void moveDoor(bool _direction)
{
  if(_direction)
  {
    if(digitalRead(PIN_R1))
    {
      digitalWrite(PIN_R1, LOW); 
      delay(300);
      Serial<<F("Delay in moveDoor +")<<_endl;
    }
     digitalWrite(PIN_R0, HIGH);
     Serial<<F("moveDoor +")<<_endl;
  }
  else
  {
    if(digitalRead(PIN_R0))
    {
      digitalWrite(PIN_R0, LOW); 
      delay(300);
      Serial<<F("Delay in moveDoor -")<<_endl;
    }
     digitalWrite(PIN_R1, HIGH);
     Serial<<F("moveDoor -")<<_endl;
  }
}

void stopDoor()
{
  digitalWrite(PIN_R0, LOW); 
  digitalWrite(PIN_R1, LOW); 
}
////////////////////////Door state machine///////////////////////////////////////

void Door_wait()
{
  if(Door.isFirstRun())
    Serial<<F("Door_wait ")<<DoorActualPos*1<<F(" ")<<DoorWantedPos<<_endl;
    DoorWantedPos=round(DoorWantedPos);
    if(DoorWantedPos<0) DoorWantedPos=0;
    if(DoorWantedPos>100) DoorWantedPos=100;
  
    Serial<<F("round(DoorActualPos): ")<<round(DoorActualPos)<<F(" ")<<DoorWantedPos<<_endl;
      
    //if(DoorActualPos==DoorWantedPos) Door.next(Door_halt);stopMixValve();
    if(DoorActualPos==DoorWantedPos) stopDoor();
    if(DoorActualPos<DoorWantedPos) Door.next(Door_moveOpen);
    if(DoorActualPos>DoorWantedPos) Door.next(Door_moveClose);
}

void Door_moveOpen()
{
  if(Door.isFirstRun())
    Serial<<F("Door_moveOpen ")<<DoorActualPos*1<<F(" ")<<DoorWantedPos<<_endl;
    moveDoor(DoorOPEN);
   // if(Door.elapsed(C[1][0]*10)) //C1.0 is in s so we need to *1000 to get value in ms.
    if(Door.elapsed(DoorTimeConstant/10)) //C1.0 is in s so we need to *1000 to get value in ms.
    {                           //then we /100 to get ms time needed to move 1% : so we *10
        DoorActualPos+=10;
        temp_str = String(DoorActualPos); //converting DoorActualPos (the variable above) to a String 
        temp_str.toCharArray(temp, temp_str.length() + 1); //packaging up the data to publish to mqtt whoa...
        //if(MQTTClient.publish(WaterVanneEauTopicPos,temp,true,LWMQTT_QOS1))
        //Serial<<F("published: Charmoisy/VanneEau/pos: ")<<round(DoorActualPos)<<F(" ")<<DoorWantedPos<<_endl;
        Door.next(Door_wait);
    }
}

void Door_moveClose()
{   
  if(Door.isFirstRun())
    Serial<<F("Door_moveClose ")<<DoorActualPos*1<<F(" ")<<DoorWantedPos<<_endl; 
    moveDoor(DoorCLOSE);
    //if(Door.elapsed(C[1][0]*10)) //C1.0 is in s so we need to *1000 to get value in ms.
    if(Door.elapsed(DoorTimeConstant/10)) //C1.0 is in s so we need to *1000 to get value in ms.
    {                           //then we /100 to get ms time needed to move 1% : so we *10
        DoorActualPos-=10;
        temp_str = String(DoorActualPos); //converting DoorActualPos (the variable above) to a String 
        temp_str.toCharArray(temp, temp_str.length() + 1); //packaging up the data to publish to mqtt whoa...
        //if(MQTTClient.publish(WaterVanneEauTopicPos,temp,true,LWMQTT_QOS1))
        //Serial<<F("published: Charmoisy/VanneEau/pos: ")<<round(DoorActualPos)<<F(" ")<<DoorWantedPos<<_endl;
        Door.next(Door_wait);
    }
}
