<h2>HenHouse V2.0</h2>
<h3>solar powered smart and automated hen house system with LoRaWan connectivity</h3><br />

<p align="center"> <img src="/Images/P_2.jpg" width="702" title="Schematics"> </p> <br /><br />

<h4>Key features</h4>
<li>Runs on an Adafruit Feather M0 board (ATSAMD21G18 ARM Cortex M0 processor, clocked at 48 MHz and at 3.3V logic) equiped with a LoRa Radio Module</li>
<li>LoraWan connectivity to TTN network based on LMIC fully compliant library</li>
<li>Entrance guillotine-style door is motorized. Two reed end-switches detect the door open/close positions</li>
<li>UP/DOWN interface buttons to manually force door opening/closing</li>
<li>Door automatically opens/closes at sunrise and sunset (as a function of an ambient light threshold, measured via the solar panel voltage output)</li>
<li>When door is moving, a buzzer makes intermitent beeps in order to let the chiken know it is bedtime</li>
<li>Measures the individual temperature of 4 separate nesting boxes (aim is to detect when a new egg was layed) + computes an average</li>
<li>Runs on a 3.7V 4400mAh Lithium Ion Battery Pack and a 6V, 1W solar panel</li>
<li>Goes to deep sleep until UP/DOWN buttons are pressed. 80µA Current consumption in deep sleep mode. </li>
<li>broadcasts over LoraWan every three minutes the systems’ metrics (temperatures, ambient light level, door position, battery level and charging status), received by home automation system in JSON format via MQTT broker</li><br /><br />

In order to broadcast data to the TTN network, you need to register an account as well as the Feather board. <a title="https://learn.adafruit.com/the-things-network-for-feather?view=all" href="https://learn.adafruit.com/the-things-network-for-feather?view=all">Detailed instructions here</a>.<br /><br />


<h4>Hardware</h4>
<li>x1 <a title="https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/downloads" href="https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/downloads">Adafruit Feather M0 LoRa Radio (RFM9x)</a></li>
<li>x1 <a title="https://www.adafruit.com/product/390" href="https://www.adafruit.com/product/390">Solar Lithium Ion/Polymer charger</a></li>
<li>x1 <a title="https://www.sparkfun.com/products/14451" href="https://www.sparkfun.com/products/14451">TB6612FNG</a> motor driver</li>
<li>x1 <a title="https://www.adafruit.com/product/3809" href="https://www.adafruit.com/product/3809">6V 1W Solar Panel</a></li>
<li>x1 <a title="https://www.adafruit.com/product/354" href="https://www.adafruit.com/product/354">Lithium Ion Battery Pack - 3.7V 4400mAh </a></li>
<li>x2 <a title="https://www.adafruit.com/product/375" href="https://www.adafruit.com/product/375">magnetic contact switch (door sensors) </a></li>
<li>x1 <a title="https://fr.banggood.com/DC-3V-6V-DC-1120-Gear-Motor-TT-Motor-for-Arduino-Smart-Car-Robot-DIY-p-1260117.html?gmcCountry=FR&currency=EUR&createTmp=1&utm_source=googleshopping&utm_medium=cpc_bgcs&utm_content=frank&utm_campaign=frank-ssc-frg-all-newcustom-ncv90-0420-19cov&ad_id=432079764870&gclid=CjwKCAjwqJ_1BRBZEiwAv73uwHZhEckRz5Eav4Dp6Mn5L4VXrPPJgIvCz8bU5mNYyqQY7Lxo8AMxjRoCXJcQAvD_BwE&cur_warehouse=CN" href="https://fr.banggood.com/DC-3V-6V-DC-1120-Gear-Motor-TT-Motor-for-Arduino-Smart-Car-Robot-DIY-p-1260117.html?gmcCountry=FR&currency=EUR&createTmp=1&utm_source=googleshopping&utm_medium=cpc_bgcs&utm_content=frank&utm_campaign=frank-ssc-frg-all-newcustom-ncv90-0420-19cov&ad_id=432079764870&gclid=CjwKCAjwqJ_1BRBZEiwAv73uwHZhEckRz5Eav4Dp6Mn5L4VXrPPJgIvCz8bU5mNYyqQY7Lxo8AMxjRoCXJcQAvD_BwE&cur_warehouse=CN">motorreductor </a></li>
<li>x1 <a title="https://www.banggood.com/DC-2V-24V-To-5V-28V-2A-Step-Up-Boost-Converter-Power-Supply-Module-Adjustable-Regulator-Board-p-1566600.html?rmmds=search&cur_warehouse=CN" href="https://www.banggood.com/DC-2V-24V-To-5V-28V-2A-Step-Up-Boost-Converter-Power-Supply-Module-Adjustable-Regulator-Board-p-1566600.html?rmmds=search&cur_warehouse=CN">DC-DC converter 5V output</a></li>




<p align="center"> <img src="/Hardware/HenHouse schematics2.png" width="702" title="Schematics"> </p> <br /><br />
<p align="center"> <img src="/Images/HenHouseMotorizedDoor.JPG" width="702" title="Schematics"> </p> <br /><br />
<p align="center"> <img src="/Images/HenHouseElec1.JPG" width="702" title="Schematics"> </p> <br /><br />
<p align="center"> <img src="/Images/HenHouseView.png" width="702" title="Schematics"> </p> <br /><br />