/*
Updates:
2.0
Started from scratch

2.1
Serial always only outputs when relay change state

2.2
Recirculating pump stays on for a few minutes after being used
Else in switch if statement ended too early
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
*/

/*
A0: Sump top
A1: Sump bottom
A2: Storage top
A3: Storage bottom
A4: Tank top
A5: Tank bottom

5V: Relay VCC, Switches 1-4
GND: Relay GND, Switches 1-4, all 6 sensors

13: Power sensors
3: Switch 1: Turn off automation system (all water sensors off). Allows for manual controls only.
4: Switch 2: Activate fertilizer pumps, acid timer, and recirculation pump.
5: Switch 3: Activates recirculating pump.
6: Switch 4: Activates manual acid relay and recirculating pump
7: Relay 1: Recirculating pump
8: Relay 2: Sump
9: Relay 3: Storage
10: Relay 4: Solenoid valve for city water
11: Relay 5: Fertilizer pumps
12: Relay 6: Outlet for acid intermittent timer
2: Relay 7: Manual acid
*/

#include <avr/wdt.h>
#include <DS3231.h>

int sens = 30;              //Sets sensor sensitivity
byte countdownSec = 180;    //No. of seconds delay for recirculation pump after it is switched off
int delayMili = 1000;		//Timer interval (milliseconds) between sampling of sensors and switches 

DS3231  rtc(SDA, SCL);

Time t;

//analog pins
int inSumpT = 0;
int inSumpB = 1;
int inStoT = 2;
int inStoB = 3;
int inTankT = 4;
int inTankB = 5;

//switch pins
byte switch1 = 0;
byte switch2 = 0;
byte switch3 = 0;
byte switch4 = 0;

//switch pins
int switch1pin = 3;
int switch2pin = 4;
int switch3pin = 5;
int switch4pin = 6;

//relay pins
int recircR = 7;
int sumpR = 8;
int stoR = 9;
int cityR = 10;
int fertR = 11;
int acidTimeR = 12;
int acid2R = 2;

//pin to provide power
int power = 13;

//hold analog values
int sumpT = 0;
int sumpB = 0;
int stoT = 0;
int stoB = 0;
int tankT = 0;
int tankB = 0;

int sumpT1 = 0;
int sumpT2 = 0;
int sumpT3 = 0;
int sumpB1 = 0;
int sumpB2 = 0;
int sumpB3 = 0;
int stoT1 = 0;
int stoT2 = 0;
int stoT3 = 0;
int stoB1 = 0;
int stoB2 = 0;
int stoB3 = 0;
int tankT1 = 0;
int tankT2 = 0;
int tankT3 = 0;
int tankB1 = 0;
int tankB2 = 0;
int tankB3 = 0;

//keep track of relay state
byte sumpS = 0;
byte stoS = 0;
byte cityS = 0;
byte groupS = 0;
byte recircSM = 0;

//keep track if in manual mode or not
byte man = 0;

byte recircS = 0;
byte start = 0;

int hour;
int min;
int sec;
int date;
int month;

unsigned long secondsAlive;
unsigned long secondsStarted;


void setup()
{
	Serial.begin(9600);
	rtc.begin();

	rtc.setTime(0, 0, 0);  // Set the time (24hr format): hour, min, sec
	rtc.setDate(1, 1, 2016);   // Set the date to January 1st, 2016

	//Set all relay pins high so they don't turn on at boot
	digitalWrite(recircR, HIGH);
	digitalWrite(sumpR, HIGH);
	digitalWrite(stoR, HIGH);
	digitalWrite(cityR, HIGH);
	digitalWrite(fertR, HIGH);
	digitalWrite(acidTimeR, HIGH);
	digitalWrite(acid2R, HIGH);

	pinMode(power, OUTPUT);
	pinMode(switch1, INPUT);
	pinMode(switch2, INPUT);
	pinMode(switch3, INPUT);
	pinMode(switch4, INPUT);
	pinMode(recircR, OUTPUT);
	pinMode(sumpR, OUTPUT);
	pinMode(stoR, OUTPUT);
	pinMode(cityR, OUTPUT);
	pinMode(fertR, OUTPUT);
	pinMode(acidTimeR, OUTPUT);
	pinMode(acid2R, OUTPUT);

	//watchdog
	wdt_enable(WDTO_4S);
}

void loop()
{
	wdt_reset(); //watchdog

	t = rtc.getTime();

	hour = t.hour;
	min = t.min;
	sec = t.sec;
	date = t.date;
	month = t.mon;

	secondsAlive = (month * 31 * 24 * 60 * 60) + (date * 24 * 60 * 60) + (hour * 60 * 60) + (min * 60) + sec;

	switch1 = digitalRead(switch1pin);
	switch2 = digitalRead(switch2pin);
	switch3 = digitalRead(switch3pin);
	switch4 = digitalRead(switch4pin);

	if (switch1 == HIGH && switch2 == LOW && switch3 == LOW && switch4 == LOW) {

		//if re-entering automated system
		if (man == 1) {
			digitalWrite(sumpR, HIGH);
			digitalWrite(stoR, HIGH);
			digitalWrite(cityR, HIGH);
			digitalWrite(fertR, HIGH);
			digitalWrite(acidTimeR, HIGH);
			digitalWrite(acid2R, HIGH);

			Serial.println("Exited manual mode, reset relays.");
			man = 0;
		}

		//sample sensors
		digitalWrite(power, HIGH);
		sumpT1 = analogRead(inSumpT);
		sumpB1 = analogRead(inSumpB);
		stoT1 = analogRead(inStoT);
		stoB1 = analogRead(inStoB);
		tankT1 = analogRead(inTankT);
		tankB1 = analogRead(inTankB);
		digitalWrite(power, LOW);
		delay(200);
		digitalWrite(power, HIGH);
		sumpT2 = analogRead(inSumpT);
		sumpB2 = analogRead(inSumpB);
		stoT2 = analogRead(inStoT);
		stoB2 = analogRead(inStoB);
		tankT2 = analogRead(inTankT);
		tankB2 = analogRead(inTankB);
		digitalWrite(power, LOW);
		delay(200);
		digitalWrite(power, HIGH);
		sumpT3 = analogRead(inSumpT);
		sumpB3 = analogRead(inSumpB);
		stoT3 = analogRead(inStoT);
		stoB3 = analogRead(inStoB);
		tankT3 = analogRead(inTankT);
		tankB3 = analogRead(inTankB);
		digitalWrite(power, LOW);

		sumpT = (sumpT1 + sumpT2 + sumpT3) / 3;
		sumpB = (sumpB1 + sumpB2 + sumpB3) / 3;
		stoT = (stoT1 + stoT2 + stoT3) / 3;
		stoB = (stoB1 + stoB2 + stoB3) / 3;
		tankT = (tankT1 + tankT2 + tankT3) / 3;
		tankB = (tankB1 + tankB2 + tankB3) / 3;

		//if one or more reading is a small number the sensor will read zero
		if (sumpT1 < sens || sumpT2 < sens || sumpT3 < sens) {
			Serial.print("sumpT changed to 0");
			sumpT = 0;
		}

		if (sumpB1 < sens || sumpB2 < sens || sumpB3 < sens) {
			Serial.print("sumpB changed to 0");
			sumpB = 0;
		}

		if (stoT1 < sens || stoT2 < sens || stoT3 < sens) {
			Serial.print("stoT changed to 0");
			stoT = 0;
		}

		if (stoB1 < sens || stoB2 < sens || stoB3 < sens) {
			Serial.print("stoB changed to 0");
			stoB = 0;
		}

		if (tankT1 < sens || tankT2 < sens || tankT3 < sens) {
			Serial.print("tankT changed to 0");
			tankT = 0;
		}

		if (tankB1 < sens || tankB2 < sens || tankB3 < sens) {
			Serial.print("tankB changed to 0");
			tankB = 0;
		}

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
		Serial.print('\n');

		//Reminder how to set sensitivity
		// x==0    ==    x<sens             ||         x!==0    ==    x>sens

		////Sump Relay(2)////
		if (sumpT > sens && stoT < sens && sumpS == 0) {
			Serial.println("Sump top triggered and storage not full.");
			sumpS = 1;
			digitalWrite(sumpR, LOW);
		}

		if (sumpS == 1 && sumpB < sens || sumpS == 1 && stoT > sens) {
			Serial.println("Stopped sump relay");
			digitalWrite(sumpR, HIGH);
			sumpS = 0;
		}

		////Storage Relay (3)////
		//fill tank from storage
		if (tankB < sens && stoB > sens && cityS == 0 && stoS == 0) {
			Serial.println("Filling tank from storage");
			stoS = 1;
			digitalWrite(stoR, LOW);
		}

		if (stoS == 1 && tankT > sens || stoS == 1 && stoB < sens) {
			Serial.println("Stopped storage relay");
			stoS = 0;
			digitalWrite(stoR, HIGH);
		}

		////City Relay(4)////
		//fill tank from city water
		if (tankB < sens && stoS == 0 && cityS == 0) {
			Serial.println("Filling tank from city water");
			cityS = 1;
			digitalWrite(cityR, LOW);
		}


		if (cityS == 1 && tankT > sens) {
			Serial.println("Stopped city water relay");
			cityS = 0;
			digitalWrite(cityR, HIGH);
		}

		////Group Relays////
		if (stoS + cityS == 1) {

			Serial.println("Started fertilizer relay");
			Serial.println("Started interminttent acid relay");
			Serial.println("Started recirculating relay");
			
			digitalWrite(fertR, LOW);
			digitalWrite(acidTimeR, LOW);
			digitalWrite(recircR, LOW);
			groupS = 1;
		}

		if (stoS == 0 && cityS == 0 && groupS == 1) {
			
			Serial.println("Stopped fertilizer relay");
			Serial.println("Stopped interminttent acid relay");
			
			digitalWrite(fertR, HIGH);
			digitalWrite(acidTimeR, HIGH);

			//record start time
			secondsStarted = secondsAlive;

			groupS = 0;
			start = 1;
			Serial.println("Recorded start time");
		}


	}
	else
	{
		if (man == 0) {
			Serial.println("Automated system off");
		}

		//marks manual system was entered
		man = 1;

		//turns off relays if originally activated by automatic system

		digitalWrite(sumpR, HIGH);
		sumpS = 0;
		if (stoS + cityS == 1) {
			Serial.println("Stopping fertilizer, and acid when turning off automated system");
			digitalWrite(fertR, HIGH);
			digitalWrite(acidTimeR, HIGH);
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


		if (switch2 == HIGH || switch3 == HIGH || switch4 == HIGH) {
			digitalWrite(recircR, LOW);
			recircSM = 1;
		}

		if (recircSM == 1 && switch2 == LOW || recircSM == 1 && switch3 == LOW || recircSM == 1 && switch4 == LOW) {
			//record start time
			secondsStarted = secondsAlive;

			start = 1;
			recircSM = 0;
		}

		if (switch2 == HIGH) {
			digitalWrite(fertR, LOW);
			digitalWrite(acidTimeR, LOW);
		}
		else {
			digitalWrite(fertR, HIGH);
			digitalWrite(acidTimeR, HIGH);
		}


		if (switch4 == HIGH) {
			digitalWrite(acid2R, LOW);
		}
		else {
			digitalWrite(acid2R, HIGH);
		}
	}


	//turns off recircR after chosen time
	recircS = digitalRead(recircR);
	if (recircS == LOW && start == 1) {

		Serial.println("Recirculation timer: ");
		Serial.print("Seconds: ");
		Serial.println(secondsAlive - secondsStarted);

		if ((secondsAlive - secondsStarted) >= countdownSec) {
			digitalWrite(recircR, HIGH);
			start = 0;
		}
	}

	delay(delayMili);
}