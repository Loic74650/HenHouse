HenHouse: change log
=======================

v0.0.3
-------

* Added a buzzer which makes intermitent beeps when door is actuated in order to let the chiken now it is bedtime
* ditched the photo-resistor as it was unable to measure accurately enough dark nights (and as a result would shut door too early for the chicken). Now using the voltage from the solar panel to decide when to open/close door

-------

* Added debug mode printing macro
* When door is manually actuated, it now stays in given state (open or close) until the next morning/evening
* Now two thresholds for the door automated open/close, one for morning and one for evening
* Updated door state machine code, now more reactive

v0.0.1
-------

* First commit