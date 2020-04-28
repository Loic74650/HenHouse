<h2>HenHouse</h2>
<h3>solar powered smart and automated hen house system with LoRaWan connectivity (ATmega32u4)</h3>

<h4>Key features</h4>
<li>Runs on an Adafruit Feather 32u4 board (ATmega32u4) equiped with a LoRa Radio Module</li>
<li>Entrance guillotine-style door is motorized. Two reed end-switches detect the door open/close positions</li>
<li>UP/DOWN interface buttons to manually force door opening/closing</li>
<li>Door automatically opens/closes at sunrise and sunset (function of an ambient light threshold)</li>
<li>Measures temperature in 4 egg-slots (aim is to detect when a new egg was layed) + computes an average</li>
<li>Runs on a 3.7V 4400mAh Lithium Ion Battery Pack and a 6V, 1W solar panel</li>
<li>Reports all measured inputs over LoRaWan every minute or when an interface button is activated</li>
<li>Goes to DeepSleep until UP/DOWN buttons are pressed + wakes up every minute to broadcast data over LoRaWan measured inputs</li>
<li>broadcasts every 5 minutes the systems’ metrics (temperatures, ambient light level, door position, battery level and charging status), received by home automation system in JSON format via MQTT broker</li>

<h4>Hardware</h4>
<li>x1 <a title="https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module" href="https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module">Adafruit Feather 32u4 LoRa Radio (RFM9x)</a></li>
<li>x1 <a title="https://www.adafruit.com/product/390" href="https://www.adafruit.com/product/390">Solar Lithium Ion/Polymer charger</a></li>
<li>x1 <a title="https://www.sparkfun.com/products/14451" href="https://www.sparkfun.com/products/14451">TB6612FNG</a> motor driver</li>
<li>x1 <a title="https://www.adafruit.com/product/3809" href="https://www.adafruit.com/product/3809">6V 1W Solar Panel</a></li>
<li>x1 <a title="https://www.adafruit.com/product/354" href="https://www.adafruit.com/product/354">Lithium Ion Battery Pack - 3.7V 4400mAh </a></li>
<li>x1 <a title="https://www.adafruit.com/product/161" href="https://www.adafruit.com/product/161">photoresistor</a></li>
<li>x2 <a title="https://www.adafruit.com/product/375" href="https://www.adafruit.com/product/375">magnetic contact switch (door sensors) </a></li>
<li>x1 <a title="https://fr.banggood.com/DC-3V-6V-DC-1120-Gear-Motor-TT-Motor-for-Arduino-Smart-Car-Robot-DIY-p-1260117.html?gmcCountry=FR&currency=EUR&createTmp=1&utm_source=googleshopping&utm_medium=cpc_bgcs&utm_content=frank&utm_campaign=frank-ssc-frg-all-newcustom-ncv90-0420-19cov&ad_id=432079764870&gclid=CjwKCAjwqJ_1BRBZEiwAv73uwHZhEckRz5Eav4Dp6Mn5L4VXrPPJgIvCz8bU5mNYyqQY7Lxo8AMxjRoCXJcQAvD_BwE&cur_warehouse=CN" href="https://fr.banggood.com/DC-3V-6V-DC-1120-Gear-Motor-TT-Motor-for-Arduino-Smart-Car-Robot-DIY-p-1260117.html?gmcCountry=FR&currency=EUR&createTmp=1&utm_source=googleshopping&utm_medium=cpc_bgcs&utm_content=frank&utm_campaign=frank-ssc-frg-all-newcustom-ncv90-0420-19cov&ad_id=432079764870&gclid=CjwKCAjwqJ_1BRBZEiwAv73uwHZhEckRz5Eav4Dp6Mn5L4VXrPPJgIvCz8bU5mNYyqQY7Lxo8AMxjRoCXJcQAvD_BwE&cur_warehouse=CN">motorreductor </a></li>
<li>x1 <a title="https://www.banggood.com/DC-2V-24V-To-5V-28V-2A-Step-Up-Boost-Converter-Power-Supply-Module-Adjustable-Regulator-Board-p-1566600.html?rmmds=search&cur_warehouse=CN" href="https://www.banggood.com/DC-2V-24V-To-5V-28V-2A-Step-Up-Boost-Converter-Power-Supply-Module-Adjustable-Regulator-Board-p-1566600.html?rmmds=search&cur_warehouse=CN">DC-DC converter 5V output</a></li>




<p align="center"> <img src="/Hardware/HenHouse schematics.png" width="702" title="Schematics"> </p> <br /><br />