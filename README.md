<h2>HenHouse</h2>
<h2>HenHouse solar powered automation system with LoRaWan connectivity, based on ATmega32u4</h2>

<li>UP/DOWN interface buttons to control Guillotine door opening/closing</li>
<li>Door has endswitches to stop opening/closing as well as a timeout if endswitches were not reached in time</li> 
<li>Measures temperature in 4 egg-slots + in main area</li>
<li>Measures battery voltage as well as charging state (charging, done)</li>
<li>Reports all measured inputs over LoRaWan every minute or when an interface button is activated</li>
<li>Goes to DeepSleep until UP/DOWN buttons are pressed + wakes up every minute to broadcast data over LoRaWan measured inputs</li>

<h4>Wiring</h4>
<li>Plug the Adafruit solar charger STAT1 digital output (Battery charging status) to digital input pin 2 of Feather board</li>
<li>Plug the Adafruit solar charger STAT2 digital output (Battery done charging status) to digital input pin 3 of Feather board</li>
<li>Plug guillotine door endswitch UP to digital input pin 5 and GND of Feather board</li>
<li>Plug guillotine door endswitch DOWN to digital input pin 6 and GND of Feather board</li>
<li>Plug Large side door interlock switch to digital input pin 9 and GND of Feather board</li>
<li>Plug door UP button between pin 0 and GND</li>
<li>Plug door DOWN button between pin 1 and GND</li>

<p align="center"> <img src="/Hardware/HenHouse schematics.png" width="702" title="Schematics"> </p> <br /><br />