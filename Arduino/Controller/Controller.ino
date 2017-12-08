
#include <TinyGPS++.h>
#include <AltSoftSerial.h>
#include <EnableInterrupt.h>
#include <waypointClass.h>     

static const uint32_t SERIAL_PORT_BAUD = 115200;

/////

boolean auto_control = false; //sets to auto_pilot

/////

static const int TURN_TIME_LIMIT = 1000;

unsigned long startTime_1 = 0, startTime_2 = 0, startTime_3 = 0, startTime_4 = 0, startTime_5 = 0, startTime_sD = 0,
startTime_6 = 0;
unsigned long long_interval = 30000, short_interval = 1000;

unsigned long start2 = 0, start1 = 0;
boolean no_right = false, no_left = false; //Limits right turning
boolean no_interupt = false;

boolean RightWall = false, LeftWall = false; //Is something in front or not

static const int trig1_Pin = 34, echo1_Pin = 35; // Left UltraSonic Sensor Pins - Mega Pin (RX_pin = 34, TX_pin = 35)
static const int trig2_Pin = 36, echo2_Pin = 37; // Right UltraSonic Sensor Pins - Mega Pin (RX_pin = 36, TX_pin = 37)
static const uint32_t GPS_BAUD = 9600;
static const int LED_G_Pin = 5; 
static const int HORN_PIN = 42; //Pin for horn connected to relay - Mega Pin (42)
static const int opSens = A3; //Debug Pin

//**** Code provided by: http://www.instructables.com/id/Arduino-Powered-Autonomous-Vehicle/
//
#define MAX_DISTANCE_CM 600                        // Maximum distance we want to ping for (in CENTIMETERS). Maximum sensor distance is rated at 400-500cm.  
#define MAX_DISTANCE_IN (MAX_DISTANCE_CM / 2.5)    // same distance, in inches

#define FAST_SPEED 55 // 210 
#define NORMAL_SPEED 75 // 190
#define TURN_SPEED 95 // 170
#define SLOW_SPEED 135 //150
#define NO_SPEED 127
int speed = NORMAL_SPEED;
int NOW_SPEED = speed;
#define REVERSE_SPEED 170 //50

#define TURN_LEFT 1
#define TURN_RIGHT 2
#define TURN_STRAIGHT 99

int targetHeading;              // where we want to go to reach current waypoint
int currentHeading;             // where we are actually facing now
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
#define HEADING_TOLERANCE 5     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

int sonarDistanceLeft, sonarDistanceRight;
// GPS Navigation
#define GPSECHO false           // set to TRUE for GPS debugging if needed
//#define GPSECHO true           // set to TRUE for GPS debugging if needed
boolean usingInterrupt = false;
float currentLat, currentLong, targetLat, targetLong;
int distanceToTarget,            // current distance to target (current waypoint)
originalDistanceToTarget;    // distance to original waypoing when we started navigating to it


							 // Waypoints
#define WAYPOINT_DIST_TOLERANE  5   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 5          // enter the numebr of way points here (will run from 0 to (n-1))
int waypointNumber = -1;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
waypointClass waypointList[NUMBER_WAYPOINTS] = { waypointClass(30.508302, -97.832624), waypointClass(30.508085, -97.832494), waypointClass(30.507715, -97.832357), waypointClass(30.508422, -97.832760), waypointClass(30.508518,-97.832665) };


// Steering/turning 
enum directions { left = TURN_LEFT, right = TURN_RIGHT, straight = TURN_STRAIGHT };
directions turnDirection = straight, oldTurnDirection = straight;


// Object avoidance distances (in inches)
#define SAFE_DISTANCE 90
#define TURN_DISTANCE 70
#define STOP_DISTANCE 40
//
// ****

// ****
// Code Provided by: 

static const int RC_NUM_CHANNELS = 2;

static const int RC_xAxis = 0;
static const int RC_yAxis = 1;

static const int RC_xAxis_INPUT = A12; // Pin for LinearActuator Mega - (A12)
static const int RC_yAxis_INPUT = A13; // Pin for Brushed DC Motor Mega - (A13)

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

//
// ****

// **** Code provided by: http://www.instructables.com/id/Rc-Controller-for-Better-Control-Over-Arduino-Proj/
//

#define TRC_NEUTRAL 127  // neutral speed
#define TRC_LEFT 192 // turn left
#define TRC_RIGHT 63 // turn right

static const int PWM_TURN = 9; //PWM signal pin for LinearAcutator (CH3)  Mega - (A12)
static const int PWM_SPEED = 12; //PWM signal pin for Brushed DC Motor (CH1) Mega - (A13)


// Assign your channel in pins
static const int THROTTLE_IN_PIN = A5; //Main motor on CH1 
static const int STEERING_IN_PIN = A4; //Steering motor on CH3

//
// ****

TinyGPSPlus gps;

// Pins for AltSoftSerial
// TXPin = 46(7) RXPin = 48(8) UsuablePins = 44, 45
AltSoftSerial  altSerial;

void setup() {

	Serial.println("Starting");
	// put your setup code here, to run once:
	pinMode(THROTTLE_IN_PIN, INPUT);
	pinMode(STEERING_IN_PIN, INPUT);

	pinMode(RC_xAxis_INPUT, INPUT);
	pinMode(RC_yAxis_INPUT, INPUT);
	//pinMode(enA, OUTPUT);
	//pinMode(enB, OUTPUT);

	enableInterrupt(RC_xAxis_INPUT, calc_ch1, CHANGE);
	enableInterrupt(RC_yAxis_INPUT, calc_ch3, CHANGE);

	pinMode(trig1_Pin, OUTPUT);
	pinMode(echo1_Pin, INPUT_PULLUP);
	pinMode(trig2_Pin, OUTPUT);
	pinMode(echo2_Pin, INPUT_PULLUP);

	pinMode(opSens, INPUT);
	pinMode(LED_G_Pin, OUTPUT);
	pinMode(HORN_PIN, INPUT_PULLUP);
	
	pinMode(HORN_PIN, OUTPUT);

	pinMode(PWM_SPEED, OUTPUT);
	pinMode(PWM_TURN, OUTPUT);

	Serial.begin(SERIAL_PORT_BAUD);
	altSerial.begin(GPS_BAUD);

	//Serial.write("Debug mode");
}

void loop() {
	int value_op = analogRead(opSens);
	//Serial.print("Debug Pin Value = ");
	//Serial.println(value_op);

	// ****
	// Code provide by: 
	rc_read_values();

	//Serial.print(F("yAX ")); Serial.print(rc_values[RC_xAxis]); Serial.println("\t");
	//Serial.print(F("xAX ")); Serial.print(rc_values[RC_yAxis]); Serial.print("\t");

	//
	// ****

	int RCx = rc_values[RC_xAxis];
	int RCy = rc_values[RC_yAxis];

	analogWrite(LED_G_Pin, 20);
	
	while (altSerial.available() > 0)
		gps.encode(altSerial.read());

	//Serial.print(F("Sat"));
	//printInt(gps.satellites.value(), gps.satellites.isValid(), 2);

	//Serial.print(F("Lat"));
	//printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);

	//Serial.print(F("Lon"));
	//printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);

	//Serial.print(F("Spd"));
	//printFloat(gps.speed.mph(), gps.speed.isValid(), 6, 2);

	//Serial.print(F("Dst"));
	//printInt(getFrontDistance(), true, 6);

	autoHornController();
	if (auto_control == true)
		autoControl();
	
	else
	{
		rc_control(RCx, RCy);
		//checkSonar();
		//updateDisplay();
	}
	// no less than 50 ms (as per JSN Ultrasonic sensor specification)
	smartDelay(200);
}

// ****
// Code provide by: 

void calc_ch1() { calc_input(RC_xAxis, RC_xAxis_INPUT); }
void calc_ch3() { calc_input(RC_yAxis, RC_yAxis_INPUT); }

void rc_read_values() {
	noInterrupts();
	memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
	interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
	if (digitalRead(input_pin) == HIGH) {
		rc_start[channel] = micros();
	}
	else {
		uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
		rc_shared[channel] = rc_compare;
	}
}

//
// ****

// ****
// Code provided by:

static void autoControl()
{
	processGPS();
	currentHeading = readCompass();    // get our current heading
	calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles      

	// distance in front of us, move, and avoid obstacles as necessary
	checkSonar();
	moveAndAvoid();

	// update display and serial monitor    
	updateDisplay();
}

void processGPS(void)
{

	currentLat = gps.location.lat();
	currentLong = gps.location.lng();

	//const char *value = TinyGPSPlus::cardinal(gps.course.value());

	// update the course and distance to waypoint based on our new position
	distanceToWaypoint();
	courseToWaypoint();

}   // processGPS(void)

void checkSonar(void)
{
	int distInLeft = getFrontLeftDistance();
	int distInRight = getFrontRightDistance();
	if (distInLeft > 235)                                // if too far to measure, return max distance;
		distInLeft = MAX_DISTANCE_IN;
	if (distInRight > 235)                                // if too far to measure, return max distance;
		distInRight = MAX_DISTANCE_IN;
	sonarDistanceLeft = (distInLeft + sonarDistanceLeft) / 2;      // add the new value into moving average, use resulting average
	sonarDistanceRight = (distInRight + sonarDistanceRight) / 2;
} // checkSonar()

int readCompass(void)
{
	double deg = gps.course.deg();
	return (int)deg;
}  // readCompass()

   // returns distance in meters between two positions, both specified 
   // as signed decimal-degrees latitude and longitude. Uses great-circle 
   // distance computation for hypothetical sphere of radius 6372795 meters.
   // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
   // copied from TinyGPS library
int distanceToWaypoint()
{
	unsigned long distanceToTarget =
		(unsigned long)TinyGPSPlus::distanceBetween(
			currentLat,
			currentLong,
			targetLat,
			targetLong) / 1000;

	// check to see if we have reached the current waypoint
	if (distanceToTarget <= WAYPOINT_DIST_TOLERANE)
		nextWaypoint();

	return (int)distanceToTarget;
}  // distanceToWaypoint()

   // returns course in degrees (North=0, West=270) from position 1 to position 2,
   // both specified as signed decimal-degrees latitude and longitude.
   // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
   // copied from TinyGPS library
int courseToWaypoint()
{
	double couseToTarget =
		TinyGPSPlus::courseTo(
			currentLat,
			currentLong,
			targetLat,
			targetLong);

	targetHeading = (int)couseToTarget;
	return targetHeading;
}   // courseToWaypoint()

// ****

void calcDesiredTurn(void)
{
	// calculate where we need to turn to head to destination
	headingError = targetHeading - currentHeading; // error in direction

	// adjust for compass wrap
	if (headingError < -180)
		headingError += 360;
	if (headingError > 180)
		headingError -= 360;

	// calculate which way to turn to intercept the targetHeading
	if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
		turnDirection = straight;
	else if (headingError < -1)
	{
		turnDirection = left; 
	}
	else if (headingError > 1)
	{
		turnDirection = right;
	}
	else
	{
		turnDirection = straight;
	}

}  // calcDesiredTurn()

void moveAndAvoid()
{
	if (sonarDistanceLeft >= SAFE_DISTANCE && sonarDistanceRight >= SAFE_DISTANCE)       // no close objects in front of car
	{
		if (turnDirection == straight)
		{
			if( no_interupt == false)
			speed = FAST_SPEED;	
		}
		else
		{
			if (no_interupt == false)
			speed = TURN_SPEED;
		}
		setSpeed(speed);
	}

	if ((sonarDistanceLeft > TURN_DISTANCE && sonarDistanceLeft < SAFE_DISTANCE) || (sonarDistanceRight > TURN_DISTANCE && sonarDistanceRight < SAFE_DISTANCE))    // not yet time to turn, but slow down
	{
		if (no_interupt == false)
		if (turnDirection == straight)
			speed = NORMAL_SPEED;
		else
		{
			if (no_interupt == false)
			speed = TURN_SPEED;
		}
		setSpeed(speed);
	}

	if (sonarDistanceLeft <  TURN_DISTANCE || sonarDistanceRight <  TURN_DISTANCE)  // getting close, time to turn to avoid object        
	{
		setSpeed(SLOW_SPEED);

		if (LeftWall == true && RightWall != true)
		{
			if (no_interupt == false)
			turnDirection = right;
		}
		else if (RightWall == true && LeftWall != true)
		{
			if (no_interupt == false)
			turnDirection = left;
		}
		else
		{
			if (headingError <= 0)
			{
				if (no_interupt == false)
					turnDirection = left;
			}
			else
			{
				if (no_interupt == false)
					turnDirection = right;
			}
		}
		turnMotor();  // turn in the new direction
	}

	if (sonarDistanceLeft < STOP_DISTANCE && sonarDistanceRight < STOP_DISTANCE)          // too close, stop and back up
	{
		setSpeed(NO_SPEED);
		turnDirection = straight; // straighten up
		turnMotor();
		smartDelay(2000);

		Serial.println("\n\nSTOPPING\n\n");

		while (sonarDistanceLeft < TURN_DISTANCE || sonarDistanceRight < TURN_DISTANCE)       // backup until we get safe clearance
		{
			setSpeed(REVERSE_SPEED);
			autoHornController();
			processGPS();
			currentHeading = readCompass();    // get our current heading
			calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles      

			checkSonar();		// distance in front of us, move, and avoid obstacles as necessary

			// update display and serial monitor    
			updateDisplay();
			Serial.println("Reversing");
			smartDelay(70);
		}
		setSpeed(NO_SPEED);  // stop backing up
	}
} // end of IF TOO CLOSE

void nextWaypoint()
{
	waypointNumber++;
	targetLat = waypointList[waypointNumber].getLat();
	targetLong = waypointList[waypointNumber].getLong();

	if ((targetLat == 0 && targetLong == 0) || waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached? 
	{
		analogWrite(PWM_TURN, TRC_NEUTRAL);
		turnDirection = straight;
		setSpeed(NO_SPEED);
		loopForever();
	}

	processGPS();
	distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
	courseToWaypoint();

}  // nextWaypoint()

void updateDisplay(void)
{
	static unsigned long lastUpdate = millis();       // for controlling frequency of LCD updates
	unsigned long currentTime;

	// check time since last update
	currentTime = millis();
	if (lastUpdate > currentTime)   // check for time wrap around
		lastUpdate = currentTime;


	Serial.print(" GPS Fix = ");
	Serial.print((int)gps.satellites.value());
	Serial.print(F(" LAT = "));
	Serial.print(currentLat);
	Serial.print(F(" LON = "));
	Serial.println(currentLong);
	Serial.print(" Waypint LAT = ");
	Serial.print(waypointList[waypointNumber].getLat());
	Serial.print(F(" Long = "));
	Serial.println(waypointList[waypointNumber].getLong());
	Serial.print(F(" Dist = "));
	Serial.print(distanceToWaypoint());
	Serial.print(F(" Original Dist = "));
	Serial.print(originalDistanceToTarget);
	Serial.println(F(" Compass Heading = "));
	Serial.print(currentHeading);
	//Serial.print(F(" GPS Heading = "));
	//Serial.println(GPS.angle);

	//Serial.println(GPS.lastNMEA());
	
	Serial.print(F("Sonar Left = "));
	Serial.println(sonarDistanceLeft, DEC);
	Serial.print(F(" Sonar Right = "));
	Serial.println(sonarDistanceRight, DEC);
	Serial.print(F(" Spd = "));
	Serial.print(speed, DEC);
	Serial.print(F("  Target = "));
	Serial.print(targetHeading, DEC);
	Serial.print(F("  Current = "));
	Serial.print(currentHeading, DEC);
	Serial.print(F("  Error = "));
	Serial.println(headingError, DEC);
	Serial.print(F("Free Memory: "));
	Serial.println(freeRam(), DEC);
	Serial.print(F("  Direction = "));
	if (turnDirection == 99)
		Serial.println("Straight");
	else if (turnDirection == 2)
		Serial.println("Right");
	else if (turnDirection == 1)
		Serial.println("Left");
	Serial.print(F("  Old Direction = "));
	if (oldTurnDirection == 99)
		Serial.println("Straight");
	else if (oldTurnDirection == 2)
		Serial.println("Right");
	else if (oldTurnDirection == 1)
		Serial.println("Left");
	
	
}  // updateDisplay()  

void loopForever(void)
{
	while (1)
		;
}

int freeRam()   // display free memory (SRAM)
{
	extern int __heap_start, *__brkval;
	int v;
	return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
} // freeRam()

//
// ****

void turnLeft()  //turn to the left
{
	if (no_left != true)
	{
		unsigned long start = millis();

		analogWrite(PWM_TURN, TRC_LEFT);

		if (start - startTime_6 >= TURN_TIME_LIMIT)
		{
			turnDirection = straight;
			analogWrite(PWM_TURN, TRC_NEUTRAL);
			no_left = true;
			startTime_6 = start;
		}
	}
}

void turnRight() //turn to the right
{
	if (no_right != true) // linear actuator limiter
	{
		unsigned long start = millis();

		analogWrite(PWM_TURN, TRC_RIGHT);

		if (start - startTime_3 >= TURN_TIME_LIMIT)
		{
			turnDirection = straight;
			analogWrite(PWM_TURN, TRC_NEUTRAL);
			no_right = true;
			startTime_3 = start;
		}
	}
}

void setSpeed(int speed)
{
	analogWrite(PWM_SPEED, speed);
}

void turnMotor()
{
	if (turnDirection == straight)
	{
		if (oldTurnDirection == left)
		{
			no_interupt = false;
			unsigned long start = millis();
			analogWrite(PWM_TURN, TRC_RIGHT);
			
			if (start - startTime_4 >= TURN_TIME_LIMIT)
			{		
				analogWrite(PWM_TURN, TRC_NEUTRAL);
				no_interupt = false;
				no_left = false;
				startTime_4 = start;
				oldTurnDirection = straight;
			}
		}
		if (oldTurnDirection == right) // if direction was orginally right and going straight
		{
			no_interupt = false;
			unsigned long start = millis();
			analogWrite(PWM_TURN, TRC_LEFT); 

			if (start - startTime_5 >= TURN_TIME_LIMIT)
			{
				analogWrite(PWM_TURN, TRC_NEUTRAL);
				no_interupt = false;
				no_right = false;
				startTime_5 = start;
				oldTurnDirection = straight;
			}
		}
		else
			no_interupt = false;
			oldTurnDirection = straight;
	}

	else if (turnDirection == left)
	{
		if (oldTurnDirection == right)
		{
			turnDirection = straight;
			turnMotor();
			turnDirection = left;
			turnMotor();
			oldTurnDirection = left;
		}
		else
		{
			turnLeft();
			oldTurnDirection = left;
		}
	}

	else if (turnDirection == right)
	{	
		if (oldTurnDirection == left)
		{
			turnDirection = straight;
			turnMotor();
			turnDirection = right;
			turnRight();
			oldTurnDirection = right;
		}
		else
		{
			turnRight();
			oldTurnDirection = right;
		}
	}
}


static void autoHornController()
{
	digitalWrite(HORN_PIN, LOW);

	unsigned long start = millis();
	if (start - startTime_1 >= long_interval)
	{
		digitalWrite(HORN_PIN, HIGH);
		//smartDelay(100);

		startTime_1 = start;
	}
}

void hornController()
{
	digitalWrite(HORN_PIN, LOW);

	unsigned long start_2 = millis();
	if (start_2 - startTime_2 >= short_interval)
	{
		digitalWrite(HORN_PIN, HIGH);

		startTime_2 = start_2;
	}
}

// ****
//

void debug()
{
	if (millis() > 5000 && gps.charsProcessed() < 10) // uh oh
	{
		Serial.println("ERROR: not getting any GPS data!");
		// dump the strseam to Serial
		Serial.println("GPS stream dump:");
		while (true) // infinite loop
			if (altSerial.available() > 0) // any data coming in?
				Serial.write(altSerial.read());
	}
	else
	{
		Serial.print("Sentences that failed checksum=");
		Serial.println(gps.failedChecksum());

		// Testing overflow in SoftwareSerial is sometimes useful too.
		Serial.print("Soft Serial device overflowed? ");
		Serial.println(altSerial.overflow() ? "YES!" : "No");
	}
}

static void smartDelay(unsigned long ms)
{
	unsigned long start = millis();
	do
	{
		while (altSerial.available())
			gps.encode(altSerial.read());
		if (auto_control == true)
			turnMotor();

	} while (millis() - start < ms);
}

static int printInt(unsigned long val, bool valid, int len)
{
	char sz[32] = "*****************";
	if (valid)
		sprintf(sz, "%ld", val);
	sz[len] = 0;
	for (int i = strlen(sz); i<len; ++i)
		sz[i] = ' ';
	if (len > 0)
		sz[len - 1] = ' ';
	Serial.println(sz);
	smartDelay(0);

	return val;
}

static float printFloat(float val, bool valid, int len, int prec)
{
	if (!valid)
	{
		while (len-- > 1)
			Serial.print('*');
		Serial.println(' ');
	}
	else
	{
		Serial.println(val, prec);
		int vi = abs((int)val);
		int flen = prec + (val < 0.0 ? 2 : 1); // . and -
		flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
		for (int i = flen; i < len; ++i)
			Serial.print(' ');
	}
	return val;
}

static const char* printStr(const char *str, int len)
{
	int slen = strlen(str);
	for (int i = 0; i < len; ++i)
		Serial.print(i < slen ? str[i] : ' ');
	smartDelay(0);
	return str;
}

//
// ****

static int getFrontLeftDistance()
{
	long duration = 0;
	int distance = 0;

	// Clears the trigPin
	digitalWrite(trig1_Pin, LOW);
	delayMicroseconds(2);

	// Sets the trigPin on HIGH state for 10 micro seconds
	digitalWrite(trig1_Pin, HIGH);
	delayMicroseconds(12);
	digitalWrite(trig1_Pin, LOW);

	// Reads the echoPin, returns the sound wave travel time in microseconds
	// Version 2.0 requires echo pin to be pulled up to VCC. 
	// A 4.7K  to 10K resistor can be used as pull-up resistor. (Uses 10k)
	duration = pulseIn(echo1_Pin, HIGH);

	//Serial.print(" Duration Left: ");
	//Serial.println(duration);

	// Calculating the distance (cm)
	distance = (int)(duration / 74 / 2);

	if (distance <= 60)
	{
		LeftWall = true;
	}
	else
	{
		LeftWall = false;
	}

	if (distance >= 8)
	{
		if (distance <= 60 && distance >= 8)
		{
			hornController();
		}
		return distance;
	}
	smartDelay(50);
}

static int getFrontRightDistance()
{
	long duration = 0;
		
	int distance = 0;

	// Clears the trigPin
	digitalWrite(trig2_Pin, LOW);
	delayMicroseconds(2);

	// Sets the trigPin on HIGH state for 10 micro seconds
	digitalWrite(trig2_Pin, HIGH);
	delayMicroseconds(12);
	digitalWrite(trig2_Pin, LOW);

	// Reads the echoPin, returns the sound wave travel time in microseconds
	// Version 2.0 requires echo pin to be pulled up to VCC. 
	// A 4.7K  to 10K resistor can be used as pull-up resistor. (Uses 10k)
	duration = pulseIn(echo2_Pin, HIGH);

	//Serial.print(" Duration Right: ");
	//Serial.println(duration);

	// Calculating the distance (in)
	distance = (int)(duration / 74 / 2);

	if (distance <= 60)
	{
		RightWall = true;
	}
	else
	{
		RightWall = false;
	}

	if (distance > 7)
	{
		if (distance <= 60 && distance >= 8)
		{
			hornController();
		}
		return distance;
	}
	smartDelay(50);
}

static void rc_control(int xAxis, int yAxis)
{
	int DCmotor = map(yAxis, 1800, 1100, 255, 63); // Map the potentiometer value from 0 to 255
	analogWrite(PWM_SPEED, DCmotor); // Send PWM signal to L298N Enable pin
	Serial.print("CH3 :");
	Serial.print(DCmotor);
	Serial.print(" Value: ");
	Serial.print(yAxis);

	int linearActuator = map(xAxis, 1950, 1080, 63, 255); // Map the potentiometer value from 0 to 255
	analogWrite(PWM_TURN, linearActuator); // Send PWM signal to L298N Enable pin
	Serial.print(" - CH1 :");
	Serial.print(linearActuator);
	Serial.print(" Value: ");
	Serial.println(xAxis);	
}
