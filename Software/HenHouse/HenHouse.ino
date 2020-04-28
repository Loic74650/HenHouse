/* HenHouse (c) Loic74 <loic74650@gmail.com> 2020
HenHouse solar-powered intelligent controller:
UP/DOWN interface buttons to control Guillotine door opening/closing
Door has endswitches to stop opening/closing as well as a timeout if endswitches were not reached in time 
Measures temperature in 4 egg-slots + an average of the 4 sensors
Measures battery voltage as well as charging state (charging, done)
Goes to DeepSleep until UP/DOWN buttons are pressed or until watchdog wakes it up (every 5 minutes) to broadcast measured inputs data over LoRaWan 

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
 Add timeout on door opening closing in case endswitches fail. Report error over MQTT if timeout reached
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
    var DigInt = (bytes[12] << 8) | bytes[13];
         
    // Decode int to float
    decoded.Mtemp = tmInt / 100;
    decoded.S1temp = S1Int / 100;
    decoded.S2temp = S2Int / 100;
    decoded.S3temp = S3Int / 100;
    decoded.S4temp = S4Int / 100;
    decoded.lum = LumInt / 100;

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
  else
  if(port == 4)
  {
    // Decode bytes to int
    var DoorWPos = (bytes[0] << 8) | bytes[1];
    var DoorAPos = (bytes[2] << 8) | bytes[3];
 
    // Decode digital inputs
    decoded.ButtonDown = (DigInt & 1) === 0?0:1;
    decoded.ButtonUp = (DigInt & 2) === 0?0:1;
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
volatile float measuredvlum = 25.0;

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
#define InContact  0  //endswitches state when in limit position

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

//Motor controller pins
#define AIN1 A3
#define AIN2 A4
#define STBY A5

//Up and Down push buttons used to open/close Hen's guillotine door
#define PUSH_BUTTON_UP    0 //INT2
#define PUSH_BUTTON_DOWN  1 //INT3

//Analog input pin to read the Battery level
#define VLUMPIN A0

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire_A(ONE_WIRE_BUS_A);

// Pass our oneWire reference to Dallas Temperature library instance 
DallasTemperature sensors_A(&oneWire_A);

//MAC Address of DS18b20 temperature sensor (!unique to every sensor!)
//DeviceAddress DS18b20_0 = { 0x28, 0xD4, 0x68, 0x00, 0x0C, 0x00, 0x00, 0x92 };//Main temp
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
int16_t lumInt = 0;
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

  // Start up the 18DS20 library
  sensors_A.begin();
       
  // set the resolution
  sensors_A.setResolution(DS18b20_1, TEMPERATURE_RESOLUTION);
  sensors_A.setResolution(DS18b20_2, TEMPERATURE_RESOLUTION);  
  sensors_A.setResolution(DS18b20_3, TEMPERATURE_RESOLUTION);  
  sensors_A.setResolution(DS18b20_4, TEMPERATURE_RESOLUTION);
  
  //Synchronous mode
  sensors_A.setWaitForConversion(true);

  //motorized door state machine init
  Door.next(Door_wait);

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
   //update Door state engine
   Door.run();
  
   //if Button UP interrupt has fired
   if(ButtonUPPressed)
    {
      //Door setpoint at 100% (open)
      DoorWantedPos = 100.0;
    
      //Serial.println(counter);
      ButtonUPPressed = 0;
      
      // Allow wake up pin to trigger interrupt on low.
      EIFR = 3;      // cancel any existing falling interrupt (interrupt 2)
      attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_UP), ButtonUPWake, LOW);  
    }

    //if Button DOWN interrupt has fired.
    if(ButtonDWNPressed)
    {
      //Door setpoint at 0% (closed)
      DoorWantedPos = 0.0;
           
      //Serial.println(counter);
      ButtonDWNPressed = 0;
      
      // Allow wake up pin to trigger interrupt on low.
      EIFR = 4;      // cancel any existing falling interrupt (interrupt 3)
      attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_DOWN), ButtonDWNWake, LOW);  
    }

    //if watchdog interrupt has fired  
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

        //send sensors data
        LoraPublish(3);
      }
     }

     //Open door at sunrise and close it at sunset, based on a 25% ambient luminosity threshold
     if((DoorActualPos == 0) && (measuredvlum > 25.0))//Door is closed and luminosity > 25% ->open door
      DoorWantedPos = 100.0;
     else
     if((DoorActualPos == 100) && (measuredvlum < 25.0))//Door is open and luminosity < 25% ->close door
      DoorWantedPos = 0.0;
     

    // Go to sleep if door is at desired position and nothing else to do
    if((int)DoorWantedPos == DoorActualPos)
      sleep();
}

void LoraPublish(unsigned char port)
{
  //port 3 is used to send sensors data at regular intervals
  if(port ==3)
  {
      sensors_A.requestTemperatures();
      float S1 = sensors_A.getTempC(DS18b20_1);
      float S2 = sensors_A.getTempC(DS18b20_2);
      float S3 = sensors_A.getTempC(DS18b20_3);
      float S4 = sensors_A.getTempC(DS18b20_4); 
      float tm = (S1+S2+S3+S4)/4.0;
       
      //Read luminosity level
      measuredvlum = analogRead(VLUMPIN);
      measuredvlum *= 100.0;  // 100%
      measuredvlum /= 1024; // convert to %
      
      // encode float as int
      lumInt = round(measuredvlum * 100);
      tmInt = round(tm * 100);
      S1Int = round(S1 * 100);
      S2Int = round(S2 * 100);
      S3Int = round(S3 * 100);
      S4Int = round(S4 * 100);
              
      //Read the Digital inputs states and record them into a Byte to be sent in the message payload
      writeBitmap(false, digitalRead(INTERLCK), digitalRead(SwDOWN), digitalRead(SwUP), digitalRead(STAT2), digitalRead(STAT1), ButtonUPPressed, ButtonDWNPressed);
       
      Serial.print("VBat: " ); 
      Serial.print(measuredvlum);
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
      
      loraData[10] = highByte(lumInt);
      loraData[11] = lowByte(lumInt);
    
      loraData[12] = highByte(0);
      loraData[13] = lowByte(DigInputs);
  }
  else
  if(port == 4)//port 4 is used to send door opening/closing progress info only when moving
  {
      //Read the Digital inputs states and record them into a Byte to be sent in the message payload
      writeBitmap(false, digitalRead(INTERLCK), digitalRead(SwDOWN), digitalRead(SwUP), digitalRead(STAT2), digitalRead(STAT1), ButtonUPPressed, ButtonDWNPressed);
       
      Serial.print("Door wanted pos: " ); 
      Serial.print(DoorWantedPos);
      Serial.print("%%");
      Serial.print(" - Door actual pos: ");
      Serial.print(DoorActualPos);
      Serial.print("%%\t");
      Serial.print("DigInputs: ");
      Serial.println(DigInputs);

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
    
      Serial.println("Sending LoRa Data...");
      lora.sendData(loraData, sizeof(loraData), port, lora.frameCounter);
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
  if(_direction)//open
  {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      if((digitalRead(SwUP) != InContact))//open door if top magnet is not in contact
        digitalWrite(STBY, HIGH);
      else
        digitalWrite(STBY, LOW);
        
   }
  else
  {
      digitalWrite(AIN2, LOW);
      digitalWrite(AIN1, HIGH);
       if((digitalRead(SwDOWN) != InContact))//close door if bottom magnet is not in contact
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
      
    if(DoorActualPos==DoorWantedPos) stopDoor();
    if(DoorActualPos<DoorWantedPos) Door.next(Door_moveOpen);
    if(DoorActualPos>DoorWantedPos) Door.next(Door_moveClose);
    if(digitalRead(SwUP) == InContact) DoorActualPos = 100.0;
    if(digitalRead(SwDOWN) == InContact) DoorActualPos = 0.0;
}

void Door_moveOpen()
{
  if(Door.isFirstRun())
    Serial<<F("Door_moveOpen ")<<DoorActualPos*1<<F(" ")<<DoorWantedPos<<_endl;
    moveDoor(DoorOPEN);
    if(Door.elapsed(DoorTimeConstant/10)) //C1.0 is in s so we need to *1000 to get value in ms.
    {                           //then we /100 to get ms time needed to move 1% : so we *10
        DoorActualPos+=10;
        LoraPublish(4);
        Serial<<F("Door_moveOpen ")<<DoorActualPos*1<<F(" - ")<<DoorWantedPos<<_endl; 
        Door.next(Door_wait);
    }
}

void Door_moveClose()
{   
  if(Door.isFirstRun())
    Serial<<F("Door_moveClose ")<<DoorActualPos*1<<F(" ")<<DoorWantedPos<<_endl; 
    moveDoor(DoorCLOSE);
    //if(Door.elapsed(C[1][0]*10)) //C1.0 is in s so we need to *1000 to get value in ms.
    if(Door.elapsed(DoorTimeConstant/10))//door has been moving for an additional 10% increment
    {                           
        DoorActualPos-=10;
        LoraPublish(4);
        Serial<<F("Door_moveClose ")<<DoorActualPos*1<<F(" - ")<<DoorWantedPos<<_endl;
        Door.next(Door_wait);
    }
}
