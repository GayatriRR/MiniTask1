# MiniTask1
The following documents my understanding of different projects done by different people across the world.

PROJ 1: Memory Puzzle Alarm Clock (https://www.instructables.com/id/Memory-Puzzle-Alarm-Clock/)

The Alarm makes an irrestitible entry in the mornings of its user. A user can switch the alarm off only when he solves a puzzle. Only when the user enters the pattern formed by the LEDs rightly within a minute, will the alarm switch off. 

Some of the main components: Two Arduinos, DS1302(Real time clock), LCD Display, 3 LEDs and 3 buttons (of the same colours), EC11 rotary encoder and speakers.

Working of the alarm: When the alarm is switched on, we first need to set the current time, the time at which alarm should ring and the difficulty level of the puzzle. When the alarm rings, when one of the alarms are pressed, the alarm stops ringing temporarily and the LEDs produce a pattern. The pattern produced by the LED must be inputted into the alarm by the user within a minute, else the alarm will automatically start ringing after that.

Implementation: The two Arduinos and the LCD display communicate through the I2C protocol, and LEDs and buttons are attached with pull-up resistors of 330 ohm and 10K respectively. The EC11 rotatory encoder is used to change the settings of each segment of the Present Time, Alarm Time and the Difficulty Level (for eg: the minutes tab of Present Time) and the push button in the same is used to navigate to the next quantity (for eg: from minutes to seconds of the Present Time; or from seconds of Present Time to hours of Alarm Time) that needs to be changed. The alarm starts ringing at the set time, and to pause it temporarily, we could press any button. A random pattern would be generated and reflected on the LEDs, which needs to be repeated by the user on the push buttons. If the action is not done within a minute, it is sensed by the microcontroller and the alarm starts ringing again. 

Comments: The I2C protocol is best used here because more than one device will need to become the master of the system. One of the Arduino is connected to the LCD display and the rotatory encoder while the other Arduino is connected to the LEDs and push buttons.
(For generally distinguishing between protocols, https://www.seeedstudio.com/blog/2019/09/25/uart-vs-i2c-vs-spi-communication-protocols-and-uses/)


PROJ 2: Updating COVID-19 Data in Real Time (https://www.instructables.com/id/MicroPython-ProgramUpdate-Coronavirus-DiseaseCOVID/)

COVID-19 is a pandemic which is growing right in front of our eyes. The project deals with getting real time data regarding the same.

Some of the main components: ESP32, Lithium battery, toggle switch and USB cable

Implementation: Using MicroPython on UPyCraft IDE, we enable USB communication with ESP32. Through this, we will be able to enable WiFi connection after entering the appropriate wifi username and password in the code. Through the Python code, we enable taking information from the website which contains statistics related to the COVID situation across different countries. Now, we print the results for different countries from the data hat is being collected from the website.The left pin of the toggle switch is welded to the power input of the ESP32 module.The positive pole of the lithium battery is connected to the middle of the toggle switch, and the negative pole is connected to the GND of the module. The toggle switch controls the on and off of the ESP32 module. When it is switched on, it connects to the internet and the display shows the statistics.


PROJ 3: The Personal Assistant (https://www.instructables.com/id/Personal-Assistant/)

Creating a device which gives notifications enabled with voice system is the motivation of this project.

Some of the important components: NodeMcu 8266, DFPlayer Mini, Micro SD card, speaker, SPST Push button

Working of the device: The device allows the user to get retrieve notifications from Gmail, Weather, Time and the number of births and deaths in that date. A push button enables navigation across different the different functions offered.

Implementation: Different services ike Gmail, weather are offered with sub-modules like unread messages, precipitation forecast, etc. For this purpose we use the circular queue using the push button to give the input to move on to the next module on each press. The ESP8266 is not powerful enough to run hash algorithms offered by the powerful APIs of Google and Yahoo, we used tricks like using Google Atom Feed to send HTTP requests to access gmail feed and a custom file on a server to send HTTP requests to Yahoo weather to get data.
We then need to enter appropriate data in the json file for each service to function; for eg: Gmail id and password for the facility to function. The micro SD card stores the mp3 files for the voice generation and the NodeMCU decides which voice bit to play. The DFPlayer Mini plays a meaningful sentence by decoding the mp3 files. Moreover, the DF Player Mini works at 5V and the ESP8266 at 3.3V. So we can't directly serial connect them, thus we use a signal diode and a 10K resistor for a level conversion. We use the flash file system of the NodeMCU for anabling the web application to enable/disable modules; basically modify settings. This can be done by identifying the IP address assigned to the ESP2866.

Comments: As ESP2866 and ESP32 provide the same functionalities, there comes the question of which one to pick for a project. The ESP32 is a later version and thus has few additional features, but for simple projects, it would be advisable to use ESP2866. (https://makeradvisor.com/esp32-vs-esp8266/ and https://community.wia.io/d/53-esp8266-vs-esp32-what-s-the-difference) have information 
regarding the same.


PROJ 4: Automatic Video Conference with Raspberry Pi (https://www.instructables.com/id/Automatic-Video-Conference-for-Grandma-With-Raspbe/)

Some of the important components: TPA3116D2 2.0 Digital power amplifier Board, Raspberry Pi, Raspberry Pi, Micro Sd 32 GB card, USB sound card, monitor, speaker, HDMI to VGA adapter, Raspberry Pi (5V-3A) power supply, ethernet cable, microphone

Implementation: Rasbian installation, setting-up of wifi and enabling remote access of Raspberry Pi must be done and the SD card with these files must be connected to the Pi and powered on. Next, the IP address of the Raspberry Pi must be found and set up in the router for remote access of the Pi from anywhere from the internet. Establishment of the raspberry connection can be done on Putty and upgrades implemented. Remote access using RealVNC needs to be set up. Installment of Noip software to ensure that the Raspberry is always available at the same address needs to be ensured. Once the whole process of remote access establishment is done on our PC, the Pi is rebooted. Now remote access from VNC Chromium on our PC will do the trick. 


PROJ 5: Multi-channel Wifi Voltage and Current Meter (https://www.instructables.com/id/Multi-channel-Wifi-Voltage-Current-Meter/)

Some of the components used: 3 Ina260 adafruit boards, ESP32 

Implementation: The Ina260 Adafruit has a very accurate voltage and current meter packed with I2C protocol and for a display, we use he ESP32 which can also carry a webserver to present the quantities on a PC or mobile screen. The 3 Ina260 boards can be assigned their own addresses for the I2C protocol communication on A0 and A1 bits in the combination of Vcc, GND, SCL and SDA. The Ina260 must be conncted to the 3.3V of the ESP32 and the SDA and SCL to the pins 21 and 22 of the ESP32. Once the ESp32 is set up on Arduino IDE and the I2C protocol functioning checked, we create a folder named 'data' that should contain the created webpage files that shall be loaded into the ESP's RAM. We get the voltage and current measurements from the 3 boards into the ESP32 and the HTML code can plot graphs for the obtained values. The webpages are then loaded into the ESP32 and configure it as a webserver.

PROJ 6: Locker (https://www.instructables.com/id/Phone-Coffer/)

Some of the components used: Arduino, keypad, LCD display, Servo Motor, USB cables, 2 LEDs (diff colours)

Implementation: The circuit involves implementation of I2C protocol. The password for the locker can be set-up now. Now, the circuit shall be built on. The push button and LEDs are connected with the A-pins and the servo motor and the keypad to the D pins of the arduino. When the locker is empty, the green LED glows, and if it has something inside, the RED glows. The presence of an item is indicated by the push button; this is because it gets pressed by the item in the locker, which indicates the presence of an item in the locker. The servo motor takes care of the locking and unocking of the locker. When the user enters the right password; the servo motor spins 180 degrees and opens. The servo motor turns back to its original position within a certain period of time, thus it should be shut by then. If not, the door can't be closed and the password needs to be entered again to rotate the servo motor again. The LCD display shows messages like "corect password", "wrong password, try again".

PROJ 7: Electronic Horn (https://www.instructables.com/id/Electronic-Loud-Horn-Using-555-Timer/)

Some of the components used: IC 555, IC LM386, resistors, potentiometer, push button, LED, capacitors, speakers, capacitors

Implementation: The 555 acts as a astable multivibrator which produces a periodic rectangular signal with duty cycle that is decided by the ratio of the resistors. The 555 timer has one resistor, one potentiometer and a capacitor to produce the signal of required frequency; the frequency can be varied by changing the resistance of the potentiometer. Then the generated signal is sent to the L386; the amplifier. The gain is already set at 20; thus with the external resistors, we can have the gain in the range 20-200. The signal passes through another potentiometer before reaching LM386, which can vary the aplitude of the final output signal. The signal now can be given to the speakers. The push button can be used to switch on and off the speaker. The capacitors are used to remove ripples. Thus we have a speaker whose frequency and amplitude can be changed. 


PROJ 8: Build a computer (https://www.instructables.com/id/Build-a-Computer-W-Basic-Understanding-of-Electron/)

Some of the components used: LM7805C:5V Linear Regulator, Zilog Z80 Microprocessor, AT28C64B EEPROM, 74LS273 and 74HC374E octal D flipflops, 3 CD4001BE quad NOR gates, NE555 clock generator, resistors, capacitors, 1 push button, button matrix, 8 LEDs

Implementation: The EEPROM stores the program for the processor Z80 to execute. The octal flip-flop is our output device that latches the data on the data bus to its own output. We will need to change the quantities on the bus without affecting what the user sees multiple times within an instruction; the D flip-flop takes care of that. The output of the D flip-flops can't drive the LED directly, thus we use two NOR gates to act as a buffer before passing the output to the LED. The input flipflop replaces the \RESET with \EN, which helps disconnecting the outputs of the chip from the bus (tri-state output). The NE555 produces the clock for the instructions to be executed accordingly. The push button will act as the reset button. We should make sure we disable the writing ability of the EEPROM.
We connect the first 8 Address pins of the microprocessor with the corresponding Address pins of the ROM. The ROM's /CE: (Chip enable) pin is wired to the processor's pin 19 (/MREQ:memory request); and the ROM's /OE (output enable) to the processor's pin 21 (/RD:read). Now we connect the data bits to the D pins on the flip-flops. The outputs of the flip-flops are connected to the NOR gates and then connected to the LEDs to give the output. We take the input from the buttons and give it to a flip-flop and connect the flipflop's output to the data bus. Now, the ROM must be programmed accordingly for the circuit to work.


PROJ 9: Alarm Bike Lock (https://www.instructables.com/id/DIY-Alarm-Bike-Lock-Shock-Activated/)

Some of the components used: LiPo battery, TP4056 (charging board), slide switches, LP2950 regualator, MCP602 opamp, resistors and capacitors, potentiometer, CD4013 RS flipflop, IRLML6344 MOSFET, buzzer, piezoelectric disc

Implementation: Connect the LiPo battery to the TP4056 (prevents over current, over charge and over discharge). Now, the shocks to the bike lock is sensed by the piezoelectric disc which produces a few ripples in the voltage. But the deviations are not large enough; thus we use an amplifier for the same. We use the LP2950 regulator to get a stable 3.3V voltage supply which can power the buzzer and the comparator. Now, the comparator has a threshold voltage in its In2-, which is compared to the amplified version of the piezoelectric response. If the input signal is more than the threshold voltage, the comparator produces a 3.3V output. This is taken to the Set input of the RS Flipflop whose output Q is passed to the buzzer. The Flipflop ensures that the buzzer rings constantly. The reason we choose an RS Flipflop and not a D flipflop for the same is because we need a mechanism to switch off the buzzer when required. Thus, we have a push button connected to the Reset of the flipflop which pulls the Reset to 3.3V when the button is pressed. Now, incase the bicycle shakes due to wind and this sets the buzzer on, we need a mechanism to prevent this situation. For this reason; a secondary RS flipflop is used whose Set input is the same as the Set input of the primary flipflop and the Reset input of this secondary flipflop shall be the Q' of the first flipflop. the Q output is connected to a resistor and capacitor in series and the voltage between the resistor and capacitor is taken and connected to the Reset of the first flipflop. Thus when the buzzer is set on accidentally, the second flipflop also has an output that is set high and the capacitor charges. Now, the voltage of the capacitor has increased and resets the first flipflop. The buzzer stops ringing. This resets the second circuit and the capacitor also discharges. Thus the circuit gets to its initial condition. We can use the slide switch to power off the whole system.

Comment: Many such projects have been implemented with microcontrollers, but using microcontrollers is not required for this. Using an Arduino would just increase the cost of the project.


PROJ 10: Spot Welder (https://www.instructables.com/id/Super-Simple-DIY-Spot-Welder-Pen-MOT-Battery-Tab-W/)
Some of the components used: Copper lugs, mini electric drill bit, tool chuck, brass tube, silicon wire

Implementation: Drill screw holes to fit the chuck and the pipe together and also holes to fit the pipe and the chuck together for soldering later. Remove a small portion of the insulation of the gauge wire and slide it into the lug and solder over the lug to make sure that the wire stays intact. The hole in the lug also helps in this purpose. Now we find the exact length of the gauge wire required and cut the wire apart. After removing the insulation of this wire, and fit the copper tube and the chuck and solder those parts to hold them together. Taper screws are used to hold the parts together now. Now we wrap the copper tube with shrink wrap to act as insulation.


PROJ 11: Moog Style Synth (https://www.instructables.com/id/Moog-Style-Synth/)

The synth is a pulse width modulated oscillator, routed through a light-controlled resonant low pass filter. Its tonality is supplied via a PWM and a high-resonance low pass filter. The light control is brought in through the LDRs which brings in a beautiful touch to this instrument.

Some of the components used: Resistors, capacitors, CD40106 inverted Schmitt Trigger, 5 Lm358 Dual operational amplifiers, Potentiometers, LDRs, 555 Timer, BC547 transistor, optocoupler (LDR and LED facing each other in a heat shrink), 9V battery, SPDT switches, push buttons, knobs for potentiometers, audio jack

Implementation: When the push buttons are pressed, it chooses the tone as the push button closes a certain path in the circuit resulting in a corresponding resistance in the circuit. The potentiometers are used to change the value of volume (amplification), or for pulse width modulation (for the beats) or even for changing the speed of the beats (changing the frequency). The optocoupler has the LED pulsing with light, which producesa rhythmic effect in the music. There are two LDRs which can produce a change in music by placing and removing the hand from them. There is a Micro USB adapter to charge the system. Moreover, an on-off switch to turn on/off the power for the whole system. Once the power of 9V is given and the output is connected through the audio jack to the speaker, the moog is ready to produce music.


PROJ 12: Wifi Hydroponics Meter (https://www.instructables.com/id/Wifi-Hydroponics-Meter/)

Some of the components used: Adafruit Esp8266, Ezo pH circuit, pH probe, EZO conductivity circuit, K1 conductivity probe, EZO temperature circuit, PT-1200 temperature probe, 2 Electrically isolated carrier boards+ 1 non-isolated board, Male USB to male Micro USB cable, 5V-1A adapter, micro USB male to micro USB female adapter

Implementation: Download Termite which is a free RS232 where serial commands can be input and responses observed. Connect the pH probe to the BNC port of the isolated carrier board and connect the pH circuit in I2C mode with the carrier board. Once calibrated, send command 'i2c,99' which will set up i2c protocol with device address 99. Similarly, we repeat for the conductivity and the temperature circuits and set them with i2c and device addresses 100 and 102. Now the Adafruit model is connected in the I2C mode with the three circuits where the pH and the conductivity circuits are connected to the isolated boards while the temperature circuit is connected to the non-isolated board. The pH and conductivity can get affected by interference from other electronics, which we prevent using the isolated carrier boards. Moreover, they get affected by changes in tempertaure; which is taken care of from the value obtained from the temperature sensor. We use ThingSpeak channel to upload our data and make sense of it. We load the ESP2866, ThinkSpeak and EZO libraries before starting the code. Now we code and include information of wifi and Thinkspeak key in it. Once all this is done, the program can be run by setting serial communication at a baud rate of 9600. The data will get uploaded to ThinkSpeak.

Comments: Though the connection are pretty straightforward in this circuit, there are important learning outcomes that can be gained from this project. For eg: the components used and the application of electronics in Chemistry.


PROJ 13: Object counter using IR (https://www.instructables.com/id/Object-Counter-Using-IR/)

Some of the components used: CD4026BE 7 segment driver, LM358 opamp, BC547 npn transistor, push button, potentiometer, resistors, 7-segment display, IR LED, photodiode, 9V power supply

Implementation: The opamp's (which acts as a comparator) output acts as a clock for the counter (7 segment driver): each time an object is sensed, the output of the opamp produces a high output resulting in a positive edge for the counter. The potentiometer helps in changing the sensitivity of the photodiode. The transistor acts as a switch here. The push button is used to reset the circuit. The carry out from the first 7 segment driver is connected to the clock of the second driver to count the 10s digit of the counter. The drivers are connected to the 7 segment displays accordingly. The object is sensed when the photodiode can't sense the IR LED's output and gives a low output which makes the transistor act like an open circuit. Due to this, the input into the In+ terminal of the comparator is a high; resulting is a high output from the comparator.


PROJ 14: Tatoo machine power supply (https://www.hackster.io/matt-reid/custom-tattoo-machine-power-supply-49d873)

Some of the components used: Microchip pic16f88 microprocessor, Maxim Integrated DS1868 digital potentiometer,INA138 current output-current sense amplifier, 555 timers, Linear regulator with adjustable output

Implementation: As there is only one output at a time, one linear regulator is only used. To direct the regulated voltage, mosfets controlled by the microcontroller have been used (based on the input given). A push button to cut of all inputs has been used too. A digital potentiometer is controls the output from the opamp. The potentiometer takes a value as per the input given and the corresponding output is given to the amplifier as the final output desired might go up to 15-16V which the potentiometer will not be able to generate (because the supply voltage of the potentiometer is 5V). Moreover, a digital potentiometer can only handle small currents. The output current is measured using a voltage divider. This is because the PIC microcontroller can operate only upto 5V while the voltage that needs to measured can go upto around 15V. To measure the duty cycle, we use the INA138 current monitor. The gain is set high so that the signal clips to produce a signal close to a square function at the output of the INA138 everytime the coils are energised. The compartor is used to produce a true square wave. Next, the 555 timer is connected in monostable mode to clear the blips and produce a solid square wave. The produced signal is passed to the microcontroller which starts the timer to measure the on time and off time and calculate the duty cycle and frequency. 


PROJ 15: Smart Doorbell Video Intercom System (https://www.hackster.io/hackershack/smart-doorbell-video-intercom-system-e5aa61)

Some of the components used: Raspberry Pi 3, MicroUSB connecter, MicroSD card, Push button with LED, USB Microphone, Raspberry Pi LCD Screen, Raspberry Pi camera, speaker with amplifier

Implemenation: The push button cum LED component is connected to the Raspberry Pi with jumper wires. A resistor must connected to the positive terminal assigned for the LED because the LED is rated only for 1.8-2.8V. Connect the USB microphone to the Raspberry Pi. The camera is also connected to the Pi. The positive wires of the speaker and the button are connected together and to GPIO pin of the Pi and the signal and ground of the speaker is soldered to the monitor. The HDMI cable must be connected to the Raspberry Pi. Now, connect to the Pi through VNC on the laptop. Next, we need to enable the camera and micrrophone and store the settings so that the settings remain after rebooting also. Next we need to load a python code which makes sure that once the button is pressed, it makes a sound and then leads to the website with the video call link and sends an email with the meeting link (to the specified email address). Once all this done, we reboot the device and the device is ready to work.


PROJ 16: XY Laser (https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)

Some of the components used: Arduino, servo motors, joystick module, laser pointer

Implemetation: The joystick module is a combination of two potentiometers with a switch. While rotating the joystick handle, we press the switch down and rotate it, and therefore change the value of resistance in the X and Y potentiometers. We connect the Vrx and Vry pins of the joystick to the analog pins of the Arduino and the switch of the joystick to a digital input pin. The servos are connected to the digital outpins of the Arduino. When the switch is pressed and the handle of the joystick rotated, the servos get input from the Arduino to how much they should rotate according to the X and Y potentiometer values. This helps in the panning and tilting of the system holding the laser pointer. This prototype can also be used in surveillance camera applications.


PROJ 17: Water level controller using 8051 Microcontroller (https://www.electronicshub.org/water-level-controller-using-8051-microcontroller/)

Some of the components used: AT89C51 Microcontroller (based on 8051 microcontroller), 8051 Development board, capacitors, resistors, push button, LCD display, NPN transistors, relays, DC Motor, potentiometer, diode, water level probes

Implementation: There are three probes for checking water level: low, half and high and one probe connected to the supply. They are connected to the P0.0, P0.1 and P0.2 of the microcontroller through a transistor each. The output is taken from P0.7 and passed through a transistor into a relay to the DC motor. The LCD Display is connected to the P1 pins. The water level is checked and corresponding messages passed to the LCD display. If all the pins P0.0, P0.1 and P0.2 are low, LCD shows 'EMPTY' and P0.7 is made HIGH to start the motor. If only P0.0 is high, LCD shows 'LOW', and when P0.1 is also high, LCD shows 'HALF'; and all this while motor continues running. When P0.2 also becomes high, P0.7 becomes low and the motor is disconnected by the relay. The LCD displays "HIGH'. 

Comments: Although we use Arduino for all our projects, if the question of choosing between different microcontrollers or ARM processor occurs, https://www.elprocus.com/difference-between-avr-arm-8051-and-pic-microcontroller/)


PROJ 18: 
PROJ 16: Automatic Alcohol Dispensor (https://www.instructables.com/id/DIY-Automatic-Alcohol-Dispenser-No-Arduino-Needed/)

Some of the components used: Proximity sensor, DC Water Pump, PNP Transistor or MOSFET, Diode

Implementation: We let the proximity sensor on the the dispensor and connect its output to the base of the transistor so that it acts as a switch. The collector is passed as input to the DC Pump. The emitter of the transistor is connected to the Vcc. Thus when the proximity sensor senses a hand nearby, the transistor acts as a closed switch and the DC pump is made to pump the alcohol into the user's hand. 


PROJ 17: Mini Laptop (https://www.instructables.com/id/MINI-LAPTOP/)

Some of the components used: 7 inch IPS display and connector, Raspberry Pi 3, Bluetooth keyboard, 5600 Mah power supply, micro USB pin, audio system, memory card

Implementation: Connect the display to the connector and fit the connector over the Raspberry Pi's GPIO pins. Get the power source, connect it to a switch and a micro USB pin to  the Raspberry Pi's power supply. A 16 GB memory card with Rasbian installed is inserted into the Micro SD card slot of the Pi. The audio system is now attached to the audio jack of the PI. The bluetooth keyboard is attached to the system through one of the USB ports of the Raspberry Pi.


PROJ 18: Bluetooth Speaker (https://www.instructables.com/id/Worlds-Smallest-Bluetooth-Speaker/)

Some of the components used: CJMCU PAM8302 (Mono class amplifier board), speaker, bluetooth board, LIPO battery

Implementation: The speakers are connected to the load pins of the amplifier board. The bluetooth board's speaker pins are connected to the audio pins of the amplifier board. The battery shall be connected to the supply of the buetooth board. When the button on the bluetooth board is pressed, the system can be paired with our device to act our speaker.

Comments: This has an easy implementation, thus could include a feature of increasing or decreasing volume of the speaker. This could be done by varying the gain of the amplifier based on the input given. This could be done by using a potentiometer to change the resistance in the circuit and thus the gain.


PROJ 19: No contact IR thermometer (https://www.instructables.com/id/No-contact-IR-Thermometer/)

Some of the components used: MLX 90614-BCH IR thermal sensor, Arduino CH340, OLED i2c Display, Laser diode, push button

Implementation: The thermal sensor and the LED display are connected using the I2C protocol. The thermal sensor senses the temperature and sends the information to the Arduino which is reflected upon on the OLED Display. The push button acts as the power button and thus we can make use of a non-contact thermometer.

