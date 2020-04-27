# MiniTask1
The following documents my understanding of different projects done by different people across the world.

Proj 1: Memory Puzzle Alarm Clock (https://www.instructables.com/id/Memory-Puzzle-Alarm-Clock/)

The Alarm makes an irrestitible entry in the mornings of its user. A user can switch the alarm off only when he solves a puzzle. Only when the user enters the pattern formed by the LEDs rightly within a minute, will the alarm switch off. 

Some of the main components: Two Arduinos, DS1302(Real time clock), LCD Display, 3 LEDs and 3 buttons (of the same colours), EC11 rotary encoder and speakers.

Working of the alarm: When the alarm is switched on, we first need to set the current time, the time at which alarm should ring and the difficulty level of the puzzle. When the alarm rings, when one of the alarms are pressed, the alarm stops ringing temporarily and the LEDs produce a pattern. The pattern produced by the LED must be inputted into the alarm by the user within a minute, else the alarm will automatically start ringing after that.

Implementation: The two Arduinos and the LCD display communicate through the I2C protocol, and LEDs and buttons are attached with pull-up resistors of 330 ohm and 10K respectively. The EC11 rotatory encoder is used to change the settings of each segment of the Present Time, Alarm Time and the Difficulty Level (for eg: the minutes tab of Present Time) and the push button in the same is used to navigate to the next quantity (for eg: from minutes to seconds of the Present Time; or from seconds of Present Time to hours of Alarm Time) that needs to be changed. The alarm starts ringing at the set time, and to pause it temporarily, we could press any button. A random pattern would be generated and reflected on the LEDs, which needs to be repeated by the user on the push buttons. If the action is not done within a minute, it is sensed by the microcontroller and the alarm starts ringing again. 

Comments: The I2C protocol is best used here because more than one device will need to become the master of the system. One of the Arduino is connected to the LCD display and the rotatory encoder while the other Arduino is connected to the LEDs and push buttons.
(For generally distinguishing between protocols, https://www.seeedstudio.com/blog/2019/09/25/uart-vs-i2c-vs-spi-communication-protocols-and-uses/)


Proj 2: Updating COVID-19 Data in Real Time (https://www.instructables.com/id/MicroPython-ProgramUpdate-Coronavirus-DiseaseCOVID/)

COVID-19 is a pandemic which is growing right in front of our eyes. The project deals with getting real time data regarding the same.

Some of the main components: ESP32, Lithium battery, toggle switch and USB cable

Implementation: Using MicroPython on UPyCraft IDE, we enable USB communication with ESP32. Through this, we will be able to enable WiFi connection after entering the appropriate wifi username and password in the code. Through the Python code, we enable taking information from the website which contains statistics related to the COVID situation across different countries. Now, we print the results for different countries from the data hat is being collected from the website.The left pin of the toggle switch is welded to the power input of the ESP32 module.The positive pole of the lithium battery is connected to the middle of the toggle switch, and the negative pole is connected to the GND of the module. The toggle switch controls the on and off of the ESP32 module. When it is switched on, it connects to the internet and the display shows the statistics.


Proj 3: The Personal Assistant (https://www.instructables.com/id/Personal-Assistant/)

Creating a device which gives notifications enabled with voice system is the motivation of this project.

Some of the important components: NodeMcu 8266, DFPlayer Mini, Micro SD card, speaker, SPST Push button

Working of the device: The device allows the user to get retrieve notifications from Gmail, Weather, Time and the number of births and deaths in that date. A push button enables navigation across different the different functions offered.

Implementation: Different services ike Gmail, weather are offered with sub-modules like unread messages, precipitation forecast, etc. For this purpose we use the circular queue using the push button to give the input to move on to the next module on each press. The ESP8266 is not powerful enough to run hash algorithms offered by the powerful APIs of Google and Yahoo, we used tricks like using Google Atom Feed to send HTTP requests to access gmail feed and a custom file on a server to send HTTP requests to Yahoo weather to get data.
We then need to enter appropriate data in the json file for each service to function; for eg: Gmail id and password for the facility to function. The micro SD card stores the mp3 files for the voice generation and the NodeMCU decides which voice bit to play. The DFPlayer Mini plays a meaningful sentence by decoding the mp3 files. Moreover, the DF Player Mini works at 5V and the ESP8266 at 3.3V. So we can't directly serial connect them, thus we use a signal diode and a 10K resistor for a level conversion. We use the flash file system of the NodeMCU for anabling the web application to enable/disable modules; basically modify settings. This can be done by identifying the IP address assigned to the ESP2866.

Comments: As ESP2866 and ESP32 provide the same functionalities, there comes the question of which one to pick for a project. The ESP32 is a later version and thus has few additional features, but for simple projects, it would be advisable to use ESP2866. (https://makeradvisor.com/esp32-vs-esp8266/ and https://community.wia.io/d/53-esp8266-vs-esp32-what-s-the-difference) have information 
regarding the same.


Proj 4: Automatic Video Conference with Raspberry Pi (https://www.instructables.com/id/Automatic-Video-Conference-for-Grandma-With-Raspbe/)

Some of the important components: TPA3116D2 2.0 Digital power amplifier Board, Raspberry Pi, Raspberry Pi, Micro Sd 32 GB card, USB sound card, monitor, speaker, HDMI to VGA adapter, Raspberry Pi (5V-3A) power supply, ethernet cable, microphone

Implementation: Rasbian installation, setting-up of wifi and enabling remote access of Raspberry Pi must be done and the SD card with these files must be connected to the Pi and powered on. Next, the IP address of the Raspberry Pi must be found and set up in the router for remote access of the Pi from anywhere from the internet. Establishment of the raspberry connection can be done on Putty and upgrades implemented. Remote access using RealVNC needs to be set up. Installment of Noip software to ensure that the Raspberry is always available at the same address needs to be ensured. Once the whole process of remote access establishment is done on our PC, the Pi is rebooted. Now remote access from VNC Chromium on our PC will do the trick. 


Proj 5: Multi-channel Wifi Voltage and Current Meter (https://www.instructables.com/id/Multi-channel-Wifi-Voltage-Current-Meter/)

Some of the components used: 3 Ina260 adafruit boards, ESP32 

Implementation: The Ina260 Adafruit has a very accurate voltage and current meter packed with I2C protocol and for a display, we use he ESP32 which can also carry a webserver to present the quantities on a PC or mobile screen. The 3 Ina260 boards can be assigned their own addresses for the I2C protocol communication on A0 and A1 bits in the combination of Vcc, GND, SCL and SDA. The Ina260 must be conncted to the 3.3V of the ESP32 and the SDA and SCL to the pins 21 and 22 of the ESP32. Once the ESp32 is set up on Arduino IDE and the I2C protocol functioning checked, we create a folder named 'data' that should contain the created webpage files that shall be loaded into the ESP's RAM. We get the voltage and current measurements from the 3 boards into the ESP32 and the HTML code can plot graphs for the obtained values. The webpages are then loaded into the ESP32 and configure it as a webserver.

Proj 6: Locker (https://www.instructables.com/id/Phone-Coffer/)

Some of the components used: Arduino, keypad, LCD display, Servo Motor, USB cables, 2 LEDs (diff colours)

Implementation: The circuit involves implementation of I2C protocol. The password for the locker can be set-up now. Now, the circuit shall be built on. The push button and LEDs are connected with the A-pins and the servo motor and the keypad to the D pins of the arduino. When the locker is empty, the green LED glows, and if it has something inside, the RED glows. The presence of an item is indicated by the push button; this is because it gets pressed by the item in the locker, which indicates the presence of an item in the locker. The servo motor takes care of the locking and unocking of the locker. When the user enters the right password; the servo motor spins 180 degrees and opens. The servo motor turns back to its original position within a certain period of time, thus it should be shut by then. If not, the door can't be closed and the password needs to be entered again to rotate the servo motor again. The LCD display shows messages like "corect password", "wrong password, try again".

Proj 7: Electronic Horn (https://www.instructables.com/id/Electronic-Loud-Horn-Using-555-Timer/)

Some of the components used: IC 555, IC LM386, resistors, potentiometer, push button, LED, capacitors, speakers, capacitors

Implementation: The 555 acts as a astable multivibrator which produces a periodic rectangular signal with duty cycle that is decided by the ratio of the resistors. The 555 timer has one resistor, one potentiometer and a capacitor to produce the signal of required frequency; the frequency can be varied by changing the resistance of the potentiometer. Then the generated signal is sent to the L386; the amplifier. The gain is already set at 20; thus with the external resistors, we can have the gain in the range 20-200. The signal passes through another potentiometer before reaching LM386, which can vary the aplitude of the final output signal. The signal now can be given to the speakers. The push button can be used to switch on and off the speaker. The capacitors are used to remove ripples. Thus we have a speaker whose frequency and amplitude can be changed. 


Proj 8: Build a computer (https://www.instructables.com/id/Build-a-Computer-W-Basic-Understanding-of-Electron/)

Some of the components used: LM7805C:5V Linear Regulator, Zilog Z80 Microprocessor, AT28C64B EEPROM, 74LS273 and 74HC374E octal D flipflops, 3 CD4001BE quad NOR gates, NE555 clock generator, resistors, capacitors, 1 push button, button matrix, 8 LEDs

Implementation: The EEPROM stores the program for the processor Z80 to execute. The octal flip-flop is our output device that latches the data on the data bus to its own output. We will need to change the quantities on the bus without affecting what the user sees multiple times within an instruction; the D flip-flop takes care of that. The output of the D flip-flops can't drive the LED directly, thus we use two NOR gates to act as a buffer before passing the output to the LED. The input flipflop replaces the \RESET with \EN, which helps disconnecting the outputs of the chip from the bus (tri-state output). The NE555 produces the clock for the instructions to be executed accordingly. The push button will act as the reset button. We should make sure we disable the writing ability of the EEPROM.
We connect the first 8 Address pins of the microprocessor with the corresponding Address pins of the ROM. The ROM's /CE: (Chip enable) pin is wired to the processor's pin 19 (/MREQ:memory request); and the ROM's /OE (output enable) to the processor's pin 21 (/RD:read). Now we connect the data bits to the D pins on the flip-flops. The outputs of the flip-flops are connected to the NOR gates and then connected to the LEDs to give the output. We take the input from the buttons and give it to a flip-flop and connect the flipflop's output to the data bus. Now, the ROM must be programmed accordingly for the circuit to work.


Proj 9: Alarm Bike Lock (https://www.instructables.com/id/DIY-Alarm-Bike-Lock-Shock-Activated/)

Some of the components used: LiPo battery, TP4056 (charging board), slide switches, LP2950 regualator, MCP602 opamp, resistors and capacitors, potentiometer, CD4013 RS flipflop, IRLML6344 MOSFET, buzzer, piezoelectric disc

Implementation: Connect the LiPo battery to the TP4056 (prevents over current, over charge and over discharge). Now, the shocks to the bike lock is sensed by the piezoelectric disc which produces a few ripples in the voltage. But the deviations are not large enough; thus we use an amplifier for the same. We use the LP2950 regulator to get a stable 3.3V voltage supply which can power the buzzer and the comparator. Now, the comparator has a threshold voltage in its In2-, which is compared to the amplified version of the piezoelectric response. If the input signal is more than the threshold voltage, the comparator produces a 3.3V output. This is taken to the Set input of the RS Flipflop whose output Q is passed to the buzzer. The Flipflop ensures that the buzzer rings constantly. The reason we choose an RS Flipflop and not a D flipflop for the same is because we need a mechanism to switch off the buzzer when required. Thus, we have a push button connected to the Reset of the flipflop which pulls the Reset to 3.3V when the button is pressed. Now, incase the bicycle shakes due to wind and this sets the buzzer on, we need a mechanism to prevent this situation. For this reason; a secondary RS flipflop is used whose Set input is the same as the Set input of the primary flipflop and the Reset input of this secondary flipflop shall be the Q' of the first flipflop. the Q output is connected to a resistor and capacitor in series and the voltage between the resistor and capacitor is taken and connected to the Reset of the first flipflop. Thus when the buzzer is set on accidentally, the second flipflop also has an output that is set high and the capacitor charges. Now, the voltage of the capacitor has increased and resets the first flipflop. The buzzer stops ringing. This resets the second circuit and the capacitor also discharges. Thus the circuit gets to its initial condition.

Comment: Many such projects have been implemented with microcontrollers, but using microcontrollers is not required for this. Using an Arduino would just increase the cost of the project.0

Proj 10: 



Proj 10: Automatic Alcohol Dispensor (https://www.instructables.com/id/DIY-Automatic-Alcohol-Dispenser-No-Arduino-Needed/)

Some of the components used: Proximity sensor, DC Water Pump, PNP Transistor or MOSFET, Diode

Implementation: We let the proximity sensor on the the dispensor and connect its output to the base of the transistor so that it acts as a switch. The collector is passed as input to the DC Pump. The emitter of the transistor is connected to the Vcc. Thus when the proximity sensor senses a hand nearby, the transistor acts as a closed switch and the DC pump is made to pump the alcohol into the user's hand. 


Proj 11: Mini Laptop (https://www.instructables.com/id/MINI-LAPTOP/)

Some of the components used: 7 inch IPS display and connector, Raspberry Pi 3, Bluetooth keyboard, 5600 Mah power supply, micro USB pin, audio system, memory card

Implementation: Connect the display to the connector and fit the connector over the Raspberry Pi's GPIO pins. Get the power source, connect it to a switch and a micro USB pin to  the Raspberry Pi's power supply. A 16 GB memory card with Rasbian installed is inserted into the Micro SD card slot of the Pi. The audio system is now attached to the audio jack of the PI. The bluetooth keyboard is attached to the system through one of the USB ports of the Raspberry Pi.


Proj 12: Bluetooth Speaker (https://www.instructables.com/id/Worlds-Smallest-Bluetooth-Speaker/)

Some of the components used: CJMCU PAM8302 (Mono class amplifier board), speaker, bluetooth board, LIPO battery

Implementation: The speakers are connected to the load pins of the amplifier board. The bluetooth board's speaker pins are connected to the audio pins of the amplifier board. The battery shall be connected to the supply of the buetooth board. When the button on the bluetooth board is pressed, the system can be paired with our device to act our speaker.

Comments: This has an easy implementation, thus could include a feature of increasing or decreasing volume of the speaker. This could be done by varying the gain of the amplifier based on the input given. This could be done by using a potentiometer to change the resistance in the circuit and thus the gain.


Proj 13: No contact IR thermometer (https://www.instructables.com/id/No-contact-IR-Thermometer/)

Some of the components used: MLX 90614-BCH IR thermal sensor, Arduino CH340, OLED i2c Display, Laser diode, push button

Implementation: The thermal sensor and the LED display are connected using the I2C protocol. The thermal sensor senses the temperature and sends the information to the Arduino which is reflected upon on the OLED Display. The push button acts as the power button and thus we can make use of a non-contact thermometer.

