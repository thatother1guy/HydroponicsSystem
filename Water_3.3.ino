#include <Adafruit_GFX.h>
#include <avr/wdt.h>
#include <DS3231.h>
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include <math.h>

/*
Updates:
2.0
Started from scratch

2.1
Serial always only outputs when relay change state

2.2
Recirculating pump stays on for a few minutes after being used
Reduced sensor sensitivity

2.3
For Arduino Mega only. Will not work on Uno.
Added watchdog
Now used real time clock (RTC) for recirculating pump delay


2.3.1
City water valve operation removed from switch 2

2.4
Redesigned how the clock works
Now uses DS3231 library by rinkydinkelectronics
Recirculation pump stays on when changing between manual/automatic systems

2.4.1
Improved sensor sampling

2.4.1.1
Moved sensor sensitivity to a single variable
Removed sensor averaging

2.4.1.2
Added sensor averaging

3.0
Added seconds alive function
Removed unnecessary clock variables
Now constantly samples switches
Changed switch functions
Removed intermittent acid timer relay, added siphon maintenence
Partially added conductivity sensor
Fixed problem when sump relay cycles on/off when storage is full

3.1
Added pH meter
Added conductivity sensor
Added 7 segement display
Added two potentiometer and a related switch

3.1.1
Fixed problem where recirculation pump could turn off too early

3.2
Removed manual acid and fertilizer switches

3.3
Controls when pH meter samples
Switches between connecting pH meter and conductivity sensor to prevent interference
Water level sensors in tank are now digital
Sample rate is less frequent when recirculating pump is off
*/

/*
A0: Sump top
A1: Sump bottom
A2: Storage top
A3: Storage bottom
A6: Conductivity probe
A7: pHProbe
A8: pH potentiometer
A9: conductivity potentiometer

5V: Relay VCC, Switches 1-4
GND: Relay GND, Switches 1-4, all 6 sensors
23: powers conductivity sensor

13: Power sensors
28: Switch 1: Turn off automation system (all water sensors off). Allows for manual controls only. (Toggle)
26: Switch 2: Activate  acid and recirculating pump (Toggle)
24: Switch 3: Activates recirculating pump. (Toggle)
22: Switch 4: Activate fertilizer and recirculating pump
3: Switch 5: Fill with sump water (MR)
4: Switch 6: Fill with city water (MR)
5: Switch 7: Activate sump pump (MR)
6: Switch 8: Siphon maintaining pump (MR)
25: Switch 9: Changes screen from actual and to wanted value (Toggle)
7: Relay A1: Recirculating pump
8: Relay A2: Sump
9: Relay A3: Storage
10: Relay A4: Solenoid valve for city water
11: Relay A5: Fertilizer pumps
12: Relay A6: Siphon maintaining pump
2: Relay A7: Acid pump
32: Relays B1 and B2: pH disconnect 
34: Relays C1 and C2: conductivity disconnect
36: Checks if irrigation pump is on
38: tank bottom
40: tank top
*/

const int sens = 30;						//Sets sensor sensitivity - use only if have problems with water sensors
const  int countdownSec = 6;				//No. of seconds delay for recirculation pump after it is switched off
const int sampleDelayActive = 5;			//Time in seconds between sampling of sensors when recircR is running for any reason
const int sampleDelayInactive = 15;			//Time in seconds between sampling of sensors when recircR isn't running
const int siphonRunTime = 10;				//Time in seconds siphon is on
const int siphonFrequency = 86400;			//Number of seconds between automatic siphon pump cycles
const float pHDifferential = 0.1;			//Determines pH range - how far (+- pH units)) from the pH setpoint
const float conductivityDifferential = 0.1;	//Determines conductivity range - how far (mmhos/cm) from the conductivity setpoint

//analog pins
const int inSumpT = 0;
const int inSumpB = 1;
const int inStoT = 2;
const int inStoB = 3;
const int inConduct = 6;
const int pHPotPin = 8;
const int conductPotPin = 9;

//switch pins
const int switch1pin = 28;
const int switch2pin = 26;
const int switch3pin = 24;
const int switch4pin = 22;
const int switch5pin = 3;
const int switch6pin = 4;
const int switch7pin = 5;
const int switch8pin = 6;
const int switch9pin = 25;
const int inTankT = 40;
const int inTankB = 38;

const int irrPin = 36;	//pin that checks irrigation state using hardware input
int manBCirr = 0;		//manual because irriagation pump

//relay pins
const int recircR = 7;
const int sumpR = 8;
const int stoR = 9;
const int cityR = 10;
const int fertR = 11;
const int siphonR = 12;
const int acidR = 2;
const int phCut = 32;
const int conductCut = 34;

//pins to provide power
const int power = 13;
const int powerConduct = 23;
const int potPower = 27;

Adafruit_7segment display = Adafruit_7segment();
DS3231  rtc(SDA, SCL);
Time t;

//pH meter code
#define SensorPin A7            //pH meter Analog output to Arduino Analog 
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10], temp;
float pHValue;

byte pHRound;
byte wantedpHRound;
byte wantedConductRound;
byte conductRound;
int sendToDisplay;
int sendToDisplayWanted;

//switchs
byte switch1 = 0;
byte switch3 = 0;
byte switch5 = 0;
byte switch6 = 0;
byte switch7 = 0;
byte switch8 = 0;
byte switch9 = 0;
byte irrS = 0;	//irrigation state

//hold analog values
int sumpT = 0;
int sumpB = 0;
int stoT = 0;
int stoB = 0;
int tankT = 0;
int tankB = 0;
long conductVIn;
float phPotVal = 0; //0-1023
float conductPotVal = 0;

double conductivity;

//keep track of relay state
byte sumpS = 0;
byte sumpSM = 0;
byte stoS = 0;
byte stoSM = 0;  //storage state manual
byte cityS = 0;
byte citySM = 0;
byte groupS = 0;
byte fertRA = 0;
byte acidRA = 0;
byte recircSM = 0;
byte recircRS = 0;  //recirc relay state

//keep track if in manual mode or not
byte man = 0;

byte startRec = 0;
byte startSumpEnd = 0;
byte startSiphon = 0;

byte mainLoop = 0;

float wantedpH;
float wantedConduct;

unsigned long secondsAlive;
unsigned long recSecondsStarted;
unsigned long mainSecondsStarted;
unsigned long enteredMainLoop = 0;
unsigned long sumpEndTime;
unsigned long enteredSiphon = 0;
unsigned long secPassedSiphon;
unsigned long secInSiphon;
unsigned long siphonStartTime;

int secPassedMain = 0;

////************** Functions **************////
unsigned long getSecondsAlive() {
	t = rtc.getTime();

	secondsAlive = (t.mon * 31 * 24 * 60 * 60) + (t.date * 24 * 60 * 60) + (t.hour * 60 * 60) + (t.min * 60) + t.sec;
	return secondsAlive;
}

void displayCurrent(float pHValue, float conductivity) {

	pHRound = round(pHValue * 10) / 1;
	conductRound = round(conductivity * 10) / 1;


	if (pHRound > 99) {
		pHRound = 00;
	}

	if (conductRound > 99) {
		conductRound = 00;
	}

	sendToDisplay = pHRound * 100 + conductRound;

	display.drawColon(true);
	display.writeDigitNum(0, (sendToDisplay / 1000), true);
	display.writeDigitNum(1, (sendToDisplay / 100) % 10, false);
	display.writeDigitNum(3, (sendToDisplay / 10) % 10, true);
	display.writeDigitNum(4, sendToDisplay % 10, false);

	display.writeDisplay();
}

void displayWanted(byte wantedpHRound, byte wantedConductRound) {

	sendToDisplayWanted = wantedpHRound * 100 + wantedConductRound;

	display.drawColon(true);
	display.writeDigitNum(0, (sendToDisplayWanted / 1000), true);
	display.writeDigitNum(1, (sendToDisplayWanted / 100) % 10, false);
	display.writeDigitNum(3, (sendToDisplayWanted / 10) % 10, true);
	display.writeDigitNum(4, sendToDisplayWanted % 10, false);

	display.writeDisplay();
}


double getConductivity(long conductVIn) {

	//used for calibrating conductivity probe
	const double  x1 = 0.0718817394154505;
	const double  x2 = -0.000119064406246838;
	const double  x3 = 0.0000000668794816104175;
	const double  intercept = -14.2057034434784;

	conductivity = x3 * conductVIn * conductVIn * conductVIn + x2 * conductVIn  * conductVIn + x1 * conductVIn + intercept;

	if (conductivity < 0) {
		conductivity = 0;
	}

	return conductivity;
}

float pH() {

	for (int i = 0; i<10; i++)       //Get 10 sample value from the sensor for smooth the value
	{
		buf[i] = analogRead(SensorPin);
		delay(10);
	}
	for (int i = 0; i<9; i++)        //sort the analog from small to large
	{
		for (int j = i + 1; j<10; j++)
		{
			if (buf[i]>buf[j])
			{
				temp = buf[i];
				buf[i] = buf[j];
				buf[j] = temp;
			}
		}
	}
	avgValue = 0;
	for (int i = 2; i<8; i++)                      //take the average value of 6 center sample
		avgValue += buf[i];
	pHValue = (float)avgValue*5.0 / 1024 / 6; //convert the analog into millivolt
	pHValue = 3.5*pHValue;                      //convert the millivolt into pH value

	return pHValue;
}
////************** End of Functions **************////


void setup()
{
#ifndef __AVR_ATtiny85__
	Serial.begin(9600);
#endif
	display.begin(0x70);
	rtc.begin();

	rtc.setTime(0, 0, 0);  // Set the time (24hr format): hour, min, sec
	rtc.setDate(1, 1, 2016);   // Set the date to January 1st, 2016

							   //Set all relay pins high so they don't turn on at boot
	digitalWrite(recircR, HIGH);
	digitalWrite(sumpR, HIGH);
	digitalWrite(stoR, HIGH);
	digitalWrite(cityR, HIGH);
	digitalWrite(fertR, HIGH);
	digitalWrite(siphonR, HIGH);
	digitalWrite(acidR, HIGH);
	digitalWrite(potPower, HIGH); //keeps potentiometers on

	pinMode(power, OUTPUT);
	pinMode(powerConduct, OUTPUT);
	pinMode(potPower, OUTPUT);
	pinMode(switch1, INPUT);
	pinMode(switch3, INPUT);
	pinMode(switch5, INPUT);
	pinMode(switch6, INPUT);
	pinMode(switch7, INPUT);
	pinMode(switch8, INPUT);
	pinMode(irrPin, INPUT);
	pinMode(inTankT, INPUT);
	pinMode(inTankB, INPUT);
	pinMode(recircR, OUTPUT);
	pinMode(sumpR, OUTPUT);
	pinMode(stoR, OUTPUT);
	pinMode(cityR, OUTPUT);
	pinMode(fertR, OUTPUT);
	pinMode(siphonR, OUTPUT);
	pinMode(acidR, OUTPUT);
	pinMode(phCut, OUTPUT);
	pinMode(conductCut, OUTPUT);
	
	//watchdog
	wdt_enable(WDTO_4S);
}

void loop()
{
	wdt_reset(); //watchdog

	getSecondsAlive();

	switch1 = digitalRead(switch1pin);
	switch3 = digitalRead(switch3pin);
	switch5 = digitalRead(switch5pin);
	switch6 = digitalRead(switch6pin);
	switch7 = digitalRead(switch7pin);
	switch8 = digitalRead(switch8pin);
	switch9 = digitalRead(switch9pin);
	irrS = digitalRead(irrPin);
	phPotVal = analogRead(pHPotPin);
	conductPotVal = analogRead(conductPotPin);

	wantedpH = phPotVal / 511.5 + 5.0;			//wanted = potVal / (range/1023) + min value
	wantedConduct = conductPotVal / 426.25 + .4;

	wantedpHRound = round(wantedpH * 10) / 1;
	wantedConductRound = round(wantedConduct * 10) / 1;

	//ph Meter code
	static unsigned long samplingTime = millis();

	//checks to see if recircR is on
	recircRS = digitalRead(recircR);

	//////always runs after sample delay//////
	if (secPassedMain >= sampleDelayActive && recircRS == LOW || secPassedMain >= sampleDelayInactive && recircRS == HIGH)
	{
		mainLoop = 1;
		enteredMainLoop = secondsAlive;
		//sample sensors
		digitalWrite(power, HIGH);
		sumpT = analogRead(inSumpT);
		sumpB = analogRead(inSumpB);
		stoT = analogRead(inStoT);
		stoB = analogRead(inStoB);
		tankT = digitalRead(inTankT);
		tankB = digitalRead(inTankB);
		digitalWrite(power, LOW);

		//samples pH meter
		digitalWrite(phCut, LOW); //connects meter to ground so that it functions
		delay(200);
		pH();
		digitalWrite(phCut, HIGH); //disconnects pH meter

		delay(100);

		//samples conductivity sensor
		digitalWrite(conductCut, LOW);
		delay(200);							//give time for relay to change
		digitalWrite(powerConduct, HIGH);
		conductVIn = analogRead(inConduct);
		digitalWrite(powerConduct, LOW);
		digitalWrite(conductCut, HIGH);

		getConductivity(conductVIn);

		//output sensor values
		Serial.print("Sump Top: ");
		Serial.println(sumpT);
		Serial.print("Sump Bottom: ");
		Serial.println(sumpB);
		Serial.print("Storage Top: ");
		Serial.println(stoT);
		Serial.print("Storage Bottom: ");
		Serial.println(stoB);
		Serial.print("Tank Top: ");
		Serial.println(tankT);
		Serial.print("Tank Bottom: ");
		Serial.println(tankB);
		Serial.print("Conductivity: ");
		Serial.println(conductivity);
		Serial.print("pH value: ");
		Serial.println(pHValue);
		Serial.print('\n');

		//turns off adding storage water if manually started
		if (stoSM == 1 && tankT == HIGH || stoSM == 1 && stoB < sens) {
			Serial.print("Turned off adding storage water that was started manually");
			digitalWrite(stoR, HIGH);
			stoSM = 0;
		}

		//turns off adding city water if manually started
		if (citySM == 1 && tankT == HIGH) {
			Serial.print("Turned off adding city water that was started manually");
			digitalWrite(cityR, HIGH);
			citySM = 0;
		}

		//turns off sump if started manually
		if (sumpSM == 1 && sumpB < sens || sumpSM == 1 && stoT > sens) {
			Serial.print("Turned off sump that was started manually");
			digitalWrite(sumpR, HIGH);
			sumpSM = 0;
		}

		//turns off acid due to sensor
		if (wantedpH > pHValue + pHDifferential && acidRA == 1) {
			Serial.print("Turned off acid due to sensor");
			digitalWrite(acidR, HIGH);
			acidRA = 0;
		}

		//turns off fertilizer due to sensor
		if (wantedConduct < conductivity - conductivityDifferential && fertRA == 1) {
			Serial.print("Turned off fertilizer due to sensor");
			digitalWrite(fertR, HIGH);
			fertRA = 0;
		}

	} //end of section that always runs after sample delay

	  //checks how many seconds passed since last sampled sensors
	secPassedMain = secondsAlive - enteredMainLoop;
	secPassedSiphon = secondsAlive - enteredSiphon;

	//////Runs after delay if in automatic mode//////
	if (switch1 == HIGH && mainLoop == 1)
	{
		//if re-entering automated system because switch is flipped or irrigation pump is turned off
		if (man == 1 || manBCirr == 1 && irrS == LOW) {
			Serial.println("Exited manual mode");
			man = 0;
		}

		////Sump Relay////
		if (sumpT > sens && stoT < sens && sumpS == 0) {
			Serial.println("Sump top triggered and storage not full.");
			sumpS = 1;
			digitalWrite(sumpR, LOW);
		}

		if (sumpS == 1 && sumpB < sens) {
			Serial.println("Stopped sump relay, sump empty");
			digitalWrite(sumpR, HIGH);
			sumpS = 0;
		}

		if (sumpS == 1 && stoT > sens) {
			Serial.println("Stopping sump relay soon, storage full");
			sumpEndTime = secondsAlive;
			sumpS = 0;
			startSumpEnd = 1;
		}


		////Acid Relay////
		//turns on acid due to sensor
		if (wantedpH < pHValue - pHDifferential && acidRA == 0) {
			Serial.println("Turned on acid due to sensor");
			digitalWrite(acidR, LOW);
			digitalWrite(recircR, LOW);
			groupS = 1;
			acidRA = 1;
		}

		////Rertilizer Relay////
		//turns on fertilizer due to sensor
		if (wantedConduct > conductivity + conductivityDifferential && fertRA == 0) {
			Serial.println("Turned on fertilizer due to sensor");
			digitalWrite(fertR, LOW);
			digitalWrite(recircR, LOW);
			groupS = 1;
			fertRA = 1;
		}

		////Storage Relay////
		//fill tank from storage
		if (tankB == LOW && stoB > sens && cityS == 0 && stoS == 0) {
			Serial.println("Filling tank from storage");
			digitalWrite(stoR, LOW);
			digitalWrite(recircR, LOW);
			stoS = 1;
			groupS = 1;
		}

		if (stoS == 1 && tankT == HIGH || stoS == 1 && stoB < sens) {
			Serial.println("Stopped storage relay");
			stoS = 0;
			digitalWrite(stoR, HIGH);
		}

		////City Relay////
		//fill tank from city water
		if (tankB == LOW && stoS == 0 && cityS == 0) {
			Serial.println("Filling tank from city water");
			digitalWrite(cityR, LOW);
			digitalWrite(recircR, LOW);
			cityS = 1;
			groupS = 1;
		}


		if (cityS == 1 && tankT == HIGH) {
			Serial.println("Stopped city water relay");
			cityS = 0;
			digitalWrite(cityR, HIGH);
		}

		mainLoop = 0;
	}
	//////Turns automated system off because switch is flipped or irrigation pump is on//////
	if (switch1 == LOW && man == 0 || irrS == HIGH && man == 0)
	{
		//turns off relays if originally activated by automatic system
		digitalWrite(sumpR, HIGH);
		sumpS = 0;
		if (stoS + cityS == 1 || acidRA == 1 || fertRA == 1) {
			Serial.print("Started recirc time");
			recSecondsStarted = secondsAlive;
			startRec = 1;
		}

		if (stoS == 1) {
			Serial.println("Storage Pump turned off when turning off automated system");
			digitalWrite(stoR, HIGH);
			stoS = 0;
		}
		if (cityS == 1) {
			Serial.println("City water turned off when turning off automated system");
			digitalWrite(cityR, HIGH);
			cityS = 0;
		}
		if (acidRA == 1) {
			digitalWrite(acidR, HIGH);
			acidRA = 0;
		}
		if (fertRA == 1) {
			digitalWrite(fertR, HIGH);
			fertRA = 0;
		}
		
		//says that manual mode was entered because irrigation pump is on
		if (irrS == HIGH) {
			manBCirr = 1;	
		}

		man = 1;
	}
	//////////////////////////////////////////////////////////////
	////everything below this line always runs without delay/////
	////////////////////////////////////////////////////////////

	if (switch3 == HIGH && recircSM == 0) {
//		Serial.println("Manual starting recirculation relay");
		digitalWrite(recircR, LOW);
		recircSM = 1;
	}

	if ((recircSM == 1)
		|| (groupS == 1 && stoS == 0 && cityS == 0 && fertRA == 0 && acidRA == 0)) {
		//record start time
		recSecondsStarted = secondsAlive;
		startRec = 1;
		groupS = 0;
		recircSM = 0;
	}

	//manual switches
	if (switch5 == HIGH && stoB > sens && tankT == LOW) {
		digitalWrite(stoR, LOW);
		stoSM = 1;
	}

	if (switch6 == HIGH && tankT == LOW) {
		digitalWrite(cityR, LOW);
		citySM = 1;
	}

	if (switch7 == HIGH && stoT < sens && sumpB > sens) {
		digitalWrite(sumpR, LOW);
		sumpSM = 1;
	}

	if (switch8 == HIGH && stoB > sens) {
		digitalWrite(siphonR, LOW);
		enteredSiphon = secondsAlive;
		siphonStartTime = secondsAlive;
		startSiphon = 1;
	}

	//sets what the display shows
	if (switch9 == LOW) {
		displayCurrent(pHValue, conductivity);
	}
	else {
		displayWanted(wantedpHRound, wantedConductRound);
	}

	//turns off recircR after chosen time
	if (startRec == 1) {

		//		Serial.println("Recirculation timer: ");
		//		Serial.print("Seconds: ");
		//		Serial.println(secondsAlive - recSecondsStarted);

		if ((secondsAlive - recSecondsStarted) >= countdownSec) {
			digitalWrite(recircR, HIGH);
			startRec = 0;
		}
	}

	//turns off sumpR after chosen time when stoT full
	if (startSumpEnd == 1 && secondsAlive - sumpEndTime > 15) {
		Serial.println("Stopped sump relay");
		digitalWrite(sumpR, HIGH);
		startSumpEnd = 0;
	}

	//automatically run siphon for 15sec every 24 hours when stoB in water
	if (secPassedSiphon > siphonFrequency && stoB > sens && startSiphon == 0) {
		enteredSiphon = secondsAlive;
		siphonStartTime = secondsAlive;
		startSiphon = 1;
		digitalWrite(siphonR, LOW);
		Serial.println("Started automatic siphon");
	}

	//turns off siphon
	secInSiphon = secondsAlive - siphonStartTime;

	if (startSiphon == 1 && stoB < sens || startSiphon == 1 && secInSiphon >= siphonRunTime) {
		digitalWrite(siphonR, HIGH);
		Serial.println("Stopped siphon");
		startSiphon = 0;
	}
}