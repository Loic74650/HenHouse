# HenHouse
 HenHouse solar powered automation system with LoRaWan connectivity, based on ATmega32u4

*UP/DOWN interface buttons to control Guillotine door opening/closing
*Door has endswitches to stop opening/closing as well as a timeout if endswitches were not reached in time 
*Measures temperature in 4 egg-slots + in main area
*Measures battery voltage as well as charging state (charging, done)
*Reports all measured inputs over LoRaWan every minute or when an interface button is activated
*Goes to DeepSleep until UP/DOWN buttons are pressed + wakes up every minute to broadcast data over LoRaWan measured inputs

##Wiring
*Plug the Adafruit solar charger STAT1 digital output (Battery charging status) to digital input pin 2 of Feather board
*Plug the Adafruit solar charger STAT2 digital output (Battery done charging status) to digital input pin 3 of Feather board
*Plug guillotine door endswitch UP to digital input pin 5 and GND of Feather board
*Plug guillotine door endswitch DOWN to digital input pin 6 and GND of Feather board
*Plug Large side door interlock switch to digital input pin 9 and GND of Feather board
*Plug door UP button between pin 0 and GND
*Plug door DOWN button between pin 1 and GND