#include <Adafruit_Circuit_Playground.h>
#include <Adafruit_CircuitPlayground.h>
#include <TinyGPS++.h>
#include <AltSoftSerial.h>
#include <EnableInterrupt.h>
#include <waypointClass.h>     
#include <Servo.h>

static const uint32_t SERIAL_PORT_BAUD = 115200;

unsigned long startTime;
unsigned long interval = 30000;

static const int enA = 9, enB = 10;
static const int triGPS_RX_Pin = 34, echo_TX_Pin = 35;
static const uint32_t GPS_BAUD = 9600;
static const int LED_G_Pin = 5, HORN_PIN = 42, ONOFF_PIN = 43;
static const int opSens = A3;

//**** Code provided by: http://www.instructables.com/id/Arduino-Powered-Autonomous-Vehicle/
//
#define MAX_DISTANCE_CM 600                        // Maximum distance we want to ping for (in CENTIMETERS). Maximum sensor distance is rated at 400-500cm.  
#define MAX_DISTANCE_IN (MAX_DISTANCE_CM / 2.5)    // same distance, in inches

#define FAST_SPEED 150
#define NORMAL_SPEED 125
#define TURN_SPEED 100
#define SLOW_SPEED 75
int speed = NORMAL_SPEED;

#define TURN_LEFT 1
#define TURN_RIGHT 2
#define TURN_STRAIGHT 99

int targetHeading;              // where we want to go to reach current waypoint
int currentHeading;             // where we are actually facing now
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
#define HEADING_TOLERANCE 5     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

int sonarDistance;
								// GPS Navigation
#define GPSECHO false           // set to TRUE for GPS debugging if needed
								//#define GPSECHO true           // set to TRUE for GPS debugging if needed
boolean usingInterrupt = false;
float currentLat,
currentLong,
targetLat,
targetLong;
int distanceToTarget,            // current distance to target (current waypoint)
originalDistanceToTarget;    // distance to original waypoing when we started navigating to it


							 // Waypoints
#define WAYPOINT_DIST_TOLERANE  5   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 5          // enter the numebr of way points here (will run from 0 to (n-1))
int waypointNumber = -1;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
waypointClass waypointList[NUMBER_WAYPOINTS] = { waypointClass(30.508302, -97.832624), waypointClass(30.508085, -97.832494), waypointClass(30.507715, -97.832357), waypointClass(30.508422, -97.832760), waypointClass(30.508518,-97.832665) };


// Steering/turning 
enum directions { left = TURN_LEFT, right = TURN_RIGHT, straight = TURN_STRAIGHT };
directions turnDirection = straight;


// Object avoidance distances (in inches)
#define SAFE_DISTANCE 70
#define TURN_DISTANCE 40
#define STOP_DISTANCE 12
//
// ****

// **** Code provided by: http://www.instructables.com/id/Rc-Controller-for-Better-Control-Over-Arduino-Proj/
//

#define SRC_NEUTRAL 1500
#define SRC_MAX 2000
#define SRC_MIN 1000
#define TRC_NEUTRAL 1500
#define TRC_MAX 2000
#define TRC_MIN 1000
#define RC_DEADBAND 50
#define ERROR_center 50
#define pERROR 100  

Servo servo1, servo2;

uint16_t unSteeringMin = SRC_MIN + pERROR;
uint16_t unSteeringMax = SRC_MAX - pERROR;
uint16_t unSteeringCenter = SRC_NEUTRAL;

uint16_t unThrottleMin = TRC_MIN + pERROR;
uint16_t unThrottleMax = TRC_MAX - pERROR;
uint16_t unThrottleCenter = TRC_NEUTRAL;

#define PWM_MIN 0
#define PWM_MAX 255

#define GEAR_NONE 1
#define GEAR_IDLE 1
#define GEAR_FULL 2

#define PWM_SPEED_LEFT 10
#define PWM_SPEED_RIGHT 11
#define LEFT1 5
#define LEFT2 6
#define RIGHT1 7
#define RIGHT2 8

#define PROGRAM_PIN 9

// Assign your channel in pins
#define THROTTLE_IN_PIN 2
#define STEERING_IN_PIN 3

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;

uint8_t gThrottle = 0;
uint8_t gGear = GEAR_NONE;
uint8_t gOldGear = GEAR_NONE;

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2
#define DIRECTION_ROTATE_RIGHT 3
#define DIRECTION_ROTATE_LEFT 4

uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gDirection = DIRECTION_STOP;
uint8_t gOldDirection = DIRECTION_STOP;

static uint16_t unArrayVal[2];

#define IDLE_MAX 50

#define MODE_RUN 0


uint8_t gMode = MODE_RUN;

unsigned long pulse_time;

//
// ****

TinyGPSPlus gps;

// Pins for AltSoftSerial
// TXPin = 46(7) RXPin = 48(8) UsuablePins = 44, 45
AltSoftSerial  altSerial; 

void setup() {

  Serial.println("Starting");
  // put your setup code here, to run once:
  pinMode(triGPS_RX_Pin, OUTPUT);
  pinMode(echo_TX_Pin, INPUT); 
  pinMode(opSens, INPUT);
  pinMode(LED_G_Pin, OUTPUT);
  pinMode(HORN_PIN,INPUT_PULLUP);
  pinMode(ONOFF_PIN,INPUT_PULLUP);
  pinMode(HORN_PIN,OUTPUT);
  pinMode(ONOFF_PIN,OUTPUT);
  servo1.attach(enA);
  servo2.attach(enB);

  // ****
  //

  //attachInterrupt(0 /* INT0 = THROTTLE_IN_PIN */, calcThrottle, CHANGE);
  //attachInterrupt(1 /* INT1 = STEERING_IN_PIN */, calcSteering, CHANGE);

  pinMode(PWM_SPEED_LEFT, OUTPUT);
  pinMode(PWM_SPEED_RIGHT, OUTPUT);
  pinMode(LEFT1, OUTPUT);
  pinMode(LEFT2, OUTPUT);
  pinMode(RIGHT1, OUTPUT);
  pinMode(RIGHT2, OUTPUT);
  pinMode(12, OUTPUT);
  pulse_time = millis();
  pinMode(PROGRAM_PIN, INPUT);

  Serial.begin(SERIAL_PORT_BAUD);
  altSerial.begin(GPS_BAUD);

  //
  // ****

 //Serial.write("Debug mode");
}

void loop() {
//  int value_op = analogRead(opSens);
//  Serial.println(value_op);
  analogWrite(LED_G_Pin, 20);
  boolean auto_control = false;
  

	  while (altSerial.available() > 0)
		  gps.encode(altSerial.read());


    Serial.print(F("Sat"));
    printInt(gps.satellites.value(), gps.satellites.isValid(), 2);
    
    Serial.print(F("Lat"));
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    
    Serial.print(F("Lon"));
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);

    Serial.print(F("Spd"));
    printFloat(gps.speed.mph(), gps.speed.isValid(), 6, 2);

    Serial.print(F("Dst"));
   // printInt(getFrontDistance(), true, 6);

	autoHornController();
	if (auto_control == false)
		decodeRC_Signals();
	else
		control();

  
    
// no less than 50 ms (as per JSN Ultrasonic sensor specification)
smartDelay(50);
}


// ****
// Code provide by: http://www.instructables.com/id/Rc-Controller-for-Better-Control-Over-Arduino-Proj/

static void decodeRC_Signals()
{

	// create local variables to hold a local copies of the channel inputs
	// these are declared static so that thier values will be retained
	// between calls to loop.
	static uint16_t unThrottleIn;
	static uint16_t unSteeringIn;
	// local copy of update flags
	static uint8_t bUpdateFlags;
	// fail_safe();
	// check shared update flags to see if any channels have a new signal
	if (bUpdateFlagsShared)
	{
		noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
		pulse_time = millis();
		// take a local copy of which channels were updated in case we need to use this in the rest of loop
		bUpdateFlags = bUpdateFlagsShared;

		// in the current code, the shared values are always populated
		// so we could copy them without testing the flags
		// however in the future this could change, so lets
		// only copy when the flags tell us we can.

		if (bUpdateFlags & THROTTLE_FLAG)
		{
			unThrottleIn = unThrottleInShared;
		}

		if (bUpdateFlags & STEERING_FLAG)
		{
			unSteeringIn = unSteeringInShared;
		}

		// clear shared copy of updated flags as we have already taken the updates
		// we still have a local copy if we need to use it in bUpdateFlags
		bUpdateFlagsShared = 0;

		interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
					  // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
					  // service routines own these and could update them at any time. During the update, the
					  // shared copies may contain junk. Luckily we have our local copies to work with :-)
	}


	// do any processing from here onwards
	// only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
	// variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
	// the interrupt routines and should not be used in loop

	if (gMode == MODE_RUN)
	{
		// we are checking to see if the channel value has changed, this is indicated 
		// by the flags. For the simple pass through we don't really need this check,
		// but for a more complex project where a new signal requires significant processing
		// this allows us to only calculate new values when we have new inputs, rather than
		// on every cycle.
		if (bUpdateFlags & THROTTLE_FLAG)
		{
			// A good idea would be to check the before and after value, 
			// if they are not equal we are receiving out of range signals
			// this could be an error, interference or a transmitter setting change
			// in any case its a good idea to at least flag it to the user somehow
			unThrottleIn = constrain(unThrottleIn, unThrottleMin, unThrottleMax);

			if (unThrottleIn > (unThrottleCenter + ERROR_center))
			{
				gThrottle = map(unThrottleIn, (unThrottleCenter + ERROR_center), unThrottleMax, PWM_MIN, PWM_MAX);
				gThrottleDirection = DIRECTION_FORWARD;
			}
			else if (unThrottleIn < (unThrottleCenter - ERROR_center))
			{
				gThrottle = map(unThrottleIn, unThrottleMin, (unThrottleCenter - ERROR_center), PWM_MAX, PWM_MIN);
				gThrottleDirection = DIRECTION_REVERSE;
			}

			else
			{
				gThrottleDirection = DIRECTION_STOP;
				gThrottle = 0;
			}

			if (gThrottle < IDLE_MAX)
			{
				gGear = GEAR_IDLE;
			}
			else
			{
				gGear = GEAR_FULL;
			}
		}

		if (bUpdateFlags & STEERING_FLAG)
		{
			uint8_t throttleLeft = gThrottle;
			uint8_t throttleRight = gThrottle;

			gDirection = gThrottleDirection;

			// see previous comments regarding trapping out of range errors
			// this is left for the user to decide how to handle and flag
			unSteeringIn = constrain(unSteeringIn, unSteeringMin, unSteeringMax);

			// if idle spin on spot
			switch (gGear)
			{
			case GEAR_IDLE:
				if (unSteeringIn > (unSteeringCenter + RC_DEADBAND))
				{
					gDirection = DIRECTION_ROTATE_RIGHT;
					// use steering to set throttle
					throttleRight = throttleLeft = map(unSteeringIn, unSteeringCenter, unSteeringMax, PWM_MIN, PWM_MAX);
				}
				else if (unSteeringIn < (unSteeringCenter - RC_DEADBAND))
				{
					gDirection = DIRECTION_ROTATE_LEFT;
					// use steering to set throttle
					throttleRight = throttleLeft = map(unSteeringIn, unSteeringMin, unSteeringCenter, PWM_MAX, PWM_MIN);
				}
				break;
				// if not idle proportionally restrain inside track to turn vehicle around it
			case GEAR_FULL:
				if (unSteeringIn >(unSteeringCenter + RC_DEADBAND))
				{
					throttleLeft = map(unSteeringIn, unSteeringCenter, unSteeringMax, gThrottle, PWM_MIN);
				}
				else if (unSteeringIn < (unSteeringCenter - RC_DEADBAND))
				{
					throttleRight = map(unSteeringIn, unSteeringMin, unSteeringCenter, PWM_MIN, gThrottle);
				}

				break;
			}
			analogWrite(PWM_SPEED_LEFT, throttleLeft);
			analogWrite(PWM_SPEED_RIGHT, throttleRight);

			// Debug
			Serial.print(F("yAX"));
			printInt(unThrottleCenter, true, 6);

			Serial.print(F("xAX"));
			printInt(unSteeringIn, true, 6);
			//
		}
	}

	if ((gDirection != gOldDirection) || (gGear != gOldGear))
	{
		gOldDirection = gDirection;
		gOldGear = gGear;

		digitalWrite(LEFT1, LOW);
		digitalWrite(LEFT2, LOW);
		digitalWrite(RIGHT1, LOW);
		digitalWrite(RIGHT2, LOW);

		switch (gDirection)
		{
		case DIRECTION_FORWARD:
			digitalWrite(LEFT1, LOW);
			digitalWrite(LEFT2, HIGH);
			digitalWrite(RIGHT1, LOW);
			digitalWrite(RIGHT2, HIGH);
			break;
		case DIRECTION_REVERSE:
			digitalWrite(LEFT1, HIGH);
			digitalWrite(LEFT2, LOW);
			digitalWrite(RIGHT1, HIGH);
			digitalWrite(RIGHT2, LOW);
			break;
		case DIRECTION_ROTATE_RIGHT:
			digitalWrite(LEFT1, HIGH);
			digitalWrite(LEFT2, LOW);
			digitalWrite(RIGHT1, LOW);
			digitalWrite(RIGHT2, HIGH);
			break;
		case DIRECTION_ROTATE_LEFT:
			digitalWrite(LEFT1, LOW);
			digitalWrite(LEFT2, HIGH);
			digitalWrite(RIGHT1, HIGH);
			digitalWrite(RIGHT2, LOW);
			break;
		case DIRECTION_STOP:
			digitalWrite(LEFT1, LOW);
			digitalWrite(LEFT2, LOW);
			digitalWrite(RIGHT1, LOW);
			digitalWrite(RIGHT2, LOW);
			break;
		}
	}
		bUpdateFlags = 0;
}

void calcThrottle()
{
	// if the pin is high, its a rising edge of the signal pulse, so lets record its value
	if (digitalRead(THROTTLE_IN_PIN) == HIGH)
	{
		ulThrottleStart = micros();
	}
	else
	{
		// else it must be a falling edge, so lets get the time and subtract the time of the rising edge
		// this gives use the time between the rising and falling edges i.e. the pulse duration.
		unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
		// use set the throttle flag to indicate that a new throttle signal has been received
		bUpdateFlagsShared |= THROTTLE_FLAG;
	}
}

void calcSteering()
{
	if (digitalRead(STEERING_IN_PIN) == HIGH)
	{
		ulSteeringStart = micros();
	}
	else
	{
		unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
		bUpdateFlagsShared |= STEERING_FLAG;
	}
}

//
// ****

// ****
// Code provided by:

static void control()
{
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
	currentLat = gps.location.lat();//convertDegMinToDecDeg(gps.location.lat());
	currentLong = gps.location.lat();//convertDegMinToDecDeg(gps.location.lat());

	const char * value = TinyGPSPlus::cardinal(gps.course.deg());

	if (value == "S")            // make them signed
		currentLat = -currentLat;
	if (value == "W")
		currentLong = -currentLong;

	// update the course and distance to waypoint based on our new position
	distanceToWaypoint();
	courseToWaypoint();

}   // processGPS(void)

void checkSonar(void)
{
	int dist;

	dist = getFrontDistance();              // get distqnce in inches from the sensor
	if (dist == 0)                                // if too far to measure, return max distance;
		dist = MAX_DISTANCE_IN;
	sonarDistance = (dist + sonarDistance) / 2;      // add the new value into moving average, use resulting average
} // checkSonar()

int readCompass(void)
{
	return gps.course.deg();
}  // readCompass()

void calcDesiredTurn(void)
{
	// calculate where we need to turn to head to destination
	headingError = targetHeading - currentHeading;

	// adjust for compass wrap
	if (headingError < -180)
		headingError += 360;
	if (headingError > 180)
		headingError -= 360;

	// calculate which way to turn to intercept the targetHeading
	if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
		turnDirection = straight;
	else if (headingError < 0)
		turnDirection = left;
	else if (headingError > 0)
		turnDirection = right;
	else
		turnDirection = straight;

}  // calcDesiredTurn()

void moveAndAvoid(void)
{
	sonarDistance = getFrontDistance();
	if (sonarDistance >= SAFE_DISTANCE)       // no close objects in front of car
	{
		if (turnDirection == straight)
			speed = FAST_SPEED;
		else
			speed = TURN_SPEED;
		driveMotor->setSpeed(speed);
		driveMotor->run(FORWARD);
		turnMotor->run(turnDirection);
		return;
	}

	if (sonarDistance > TURN_DISTANCE && sonarDistance < SAFE_DISTANCE)    // not yet time to turn, but slow down
	{
		if (turnDirection == straight)
			speed = NORMAL_SPEED;
		else
		{
			speed = TURN_SPEED;
			turnMotor->run(turnDirection);      // alraedy turning to navigate
		}
		driveMotor->setSpeed(speed);
		driveMotor->run(FORWARD);
		return;
	}

	if (sonarDistance <  TURN_DISTANCE && sonarDistance > STOP_DISTANCE)  // getting close, time to turn to avoid object        
	{
		speed = SLOW_SPEED;
		driveMotor->setSpeed(speed);      // slow down
		driveMotor->run(FORWARD);
		switch (turnDirection)
		{
		case straight:                  // going straight currently, so start new turn
		{
			if (headingError <= 0)
				turnDirection = left;
			else
				turnDirection = right;
			turnMotor->run(turnDirection);  // turn in the new direction
			break;
		}
		case left:                         // if already turning left, try right
		{
			turnMotor->run(TURN_RIGHT);
			break;
		}
		case right:                       // if already turning right, try left
		{
			turnMotor->run(TURN_LEFT);
			break;
		}
		} // end SWITCH

		return;
	}


	if (sonarDistance <  STOP_DISTANCE)          // too close, stop and back up
	{
		driveMotor->run(RELEASE);            // stop 
		servo1.write(90);            // straighten up
		turnDirection = straight;
		driveMotor->setSpeed(NORMAL_SPEED);  // go back at higher speet
		driveMotor->run(BACKWARD);
		while (sonarDistance < TURN_DISTANCE)       // backup until we get safe clearance
		{
			if (GPS.parse(GPS.lastNMEA()))
				processGPS();
			currentHeading = readCompass();    // get our current heading
			calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles      
			checkSonar();
			updateDisplay();
			delay(100);
		} // while (sonarDistance < TURN_DISTANCE)
		driveMotor->run(RELEASE);        // stop backing up
		return;
	} // end of IF TOO CLOSE

}

void nextWaypoint(void)
{
	waypointNumber++;
	targetLat = waypointList[waypointNumber].getLat();
	targetLong = waypointList[waypointNumber].getLong();

	if ((targetLat == 0 && targetLong == 0) || waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached? 
	{
		//driveMotor->run(RELEASE);    // make sure we stop
		//turnMotor->run(RELEASE);
		////lcd.clear();
		////lcd.println(F("* LAST WAYPOINT *"));
		loopForever();
	}

	processGPS();
	distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
	courseToWaypoint();

}  // nextWaypoint()

   // returns distance in meters between two positions, both specified 
   // as signed decimal-degrees latitude and longitude. Uses great-circle 
   // distance computation for hypothetical sphere of radius 6372795 meters.
   // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
   // copied from TinyGPS library
int distanceToWaypoint()
{

	/*float delta = radians(currentLong - targetLong);
	float sdlong = sin(delta);
	float cdlong = cos(delta);
	float lat1 = radians(currentLat);
	float lat2 = radians(targetLat);
	float slat1 = sin(lat1);
	float clat1 = cos(lat1);
	float slat2 = sin(lat2);
	float clat2 = cos(lat2);
	delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
	delta = sq(delta);
	delta += sq(clat2 * sdlong);
	delta = sqrt(delta);
	float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
	delta = atan2(delta, denom);
	distanceToTarget = delta * 6372795;*/

	unsigned long distanceToTarget =
		(unsigned long)TinyGPSPlus::distanceBetween(
			gps.location.lat(),
			gps.location.lng(),
			targetLat,
			targetLong) / 1000;

	// check to see if we have reached the current waypoint
	if (distanceToTarget <= WAYPOINT_DIST_TOLERANE)
		nextWaypoint();

	return distanceToTarget;
}  // distanceToWaypoint()




   // returns course in degrees (North=0, West=270) from position 1 to position 2,
   // both specified as signed decimal-degrees latitude and longitude.
   // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
   // copied from TinyGPS library
int courseToWaypoint()
{
	/*float dlon = radians(targetLong - currentLong);
	float cLat = radians(currentLat);
	float tLat = radians(targetLat);
	float a1 = sin(dlon) * cos(tLat);
	float a2 = sin(cLat) * cos(tLat) * cos(dlon);
	a2 = cos(cLat) * sin(tLat) - a2;
	a2 = atan2(a1, a2);
	if (a2 < 0.0)
	{
	a2 += TWO_PI;
	}
	targetHeading = degrees(a2);*/

	double targetHeading =
		TinyGPSPlus::courseTo(
			gps.location.lat(),
			gps.location.lng(),
			targetLat,
			targetLong);

	return targetHeading;
}   // courseToWaypoint()


	// converts lat/long from Adafruit degree-minute format to decimal-degrees; requires <math.h> library
double convertDegMinToDecDeg(float degMin)
{
	/*double min = 0.0;
	double decDeg = 0.0;

	//get the minutes, fmod() requires double
	min = fmod((double)degMin, 100.0);

	//rebuild coordinates in decimal degrees
	degMin = (int)(degMin / 100);
	decDeg = degMin + (min / 60);*/

	return gps.course.deg();
}

void updateDisplay(void)
{

	static unsigned long lastUpdate = millis();       // for controlling frequency of LCD updates
	unsigned long currentTime;

	// check time since last update
	currentTime = millis();
	if (lastUpdate > currentTime)   // check for time wrap around
		lastUpdate = currentTime;

#ifdef DEBUG
	//Serial.print("GPS Fix:");
	//Serial.println((int)GPS.fix);
	Serial.print(F("LAT = "));
	Serial.print(currentLat);
	Serial.print(F(" LON = "));
	Serial.println(currentLong);
	//Serial.print("Waypint LAT ="); 
	//Serial.print(waypointList[waypointNumber].getLat());
	//Serial.print(F(" Long = "));
	//Serial.print(waypointList[waypointNumber].getLong());
	Serial.print(F(" Dist "));
	Serial.print(distanceToWaypoint());
	Serial.print(F("Original Dist "));
	Serial.println(originalDistanceToTarget);
	Serial.print(F("Compass Heading "));
	Serial.println(currentHeading);
	Serial.print(F("GPS Heading "));
	Serial.println(GPS.angle);

	//Serial.println(GPS.lastNMEA());

	//Serial.print(F("Sonar = "));
	//Serial.print(sonarDistance, DEC);
	//Serial.print(F(" Spd = "));
	//Serial.println(speed, DEC);
	//Serial.print(F("  Target = "));
	//Serial.print(targetHeading, DEC);
	//Serial.print(F("  Current = "));
	//Serial.print(currentHeading, DEC);
	//Serial.print(F("  Error = "));
	//Serial.println(headingError, DEC);
	//Serial.print(F("Free Memory: "));
	//Serial.println(freeRam(), DEC);
#endif

}  // updateDisplay()  

void loopForever(void)
{
	while (1)
		;
}
//
// ****
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

static void autoHornController()
{
	unsigned long start = millis();

	if (start - startTime >= interval) {
		digitalWrite(HORN_PIN, HIGH);

		unsigned long in_start = millis();

		if (in_start - startTime >= 100)
		{
			digitalWrite(HORN_PIN, LOW);
		}
		startTime = millis();
	}
}

void hornController(float distanceCm)
{
	unsigned long start = millis();

  if ( distanceCm <= 80 && distanceCm >= 20) {
	 digitalWrite(HORN_PIN, LOW);
     digitalWrite(HORN_PIN, HIGH);

	 if (start - startTime >= interval) {
		 digitalWrite(HORN_PIN, HIGH);

		 unsigned long in_start = millis();

		 if (in_start - startTime >= 100)
		 {
			 digitalWrite(HORN_PIN, LOW);
		 }

		 startTime = millis();
	 }
  }
  else
  {
	  digitalWrite(HORN_PIN, LOW);
  }
}

static void smartDelay(unsigned long ms)
{
	unsigned long start = millis();
	do
	{
		while (altSerial.available())
			gps.encode(altSerial.read());
	} while (millis() - start < ms);
}

static void printInt(unsigned long val, bool valid, int len)
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

static void printStr(const char *str, int len)
	{
		int slen = strlen(str);
		for (int i = 0; i < len; ++i)
			Serial.print(i < slen ? str[i] : ' ');
		smartDelay(0);
	}

static int getFrontDistance()
{
	float duration, distanceCm;

	// Clears the trigPin
	digitalWrite(triGPS_RX_Pin, LOW);
	delayMicroseconds(2);

	// Sets the trigPin on HIGH state for 10 micro seconds
	digitalWrite(triGPS_RX_Pin, HIGH);
	delayMicroseconds(10);
	digitalWrite(triGPS_RX_Pin, LOW);

	// Reads the echoPin, returns the sound wave travel time in microseconds
	// Version 2.0 requires echo pin to be pulled up to VCC. 
	// A 4.7K  to 10K resistor can be used as pull-up resistor. (Uses 10k)
	duration = pulseIn(echo_TX_Pin, HIGH);

	// Calculating the distance (cm)
	distanceCm = duration / 29 / 2;

	
	if (distanceCm <= 600 && distanceCm >= 19)
	{
		if (distanceCm <= 60 && distanceCm >= 19)
		{
			hornController(distanceCm);
		}
		return distanceCm;
	}
}

static void rc_control(int xAxis, int yAxis)
{
	int linearActuator = map(yAxis, 1828, 1160, 0, 180); // Map the potentiometer value from 0 to 255
	servo1.write( linearActuator); // Send PWM signal to L298N Enable pin
	Serial.println("1");
	Serial.println(linearActuator);
	int DCmotor = map(xAxis, 1932, 1088, 0, 180); // Map the potentiometer value from 0 to 255
	Serial.println("2");
	Serial.println(DCmotor);
	servo2.write(DCmotor); // Send PWM signal to L298N Enable pin
}