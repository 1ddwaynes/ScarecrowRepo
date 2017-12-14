#include <TinyGPS++.h>
#include <AltSoftSerial.h>
#include <EnableInterrupt.h>
#include <waypointClass.h>   

static const uint32_t SERIAL_PORT_BAUD = 115200;

/////

boolean auto_control = true; //sets to auto_pilot

/////

unsigned long startTime_1 = 0, startTime_2 = 0;
unsigned long start_interval = 4000, long_interval = 30000, short_interval = 1000;


boolean hornInUse = false;
boolean RightWall = false, LeftWall = false; //Is something in front or not

static const int trig1_Pin = 12, echo1_Pin = 11; // Left UltraSonic Sensor Pins - Mega Pin (RX_pin = 34, TX_pin = 35)
static const int trig2_Pin = 36, echo2_Pin = 37; // Right UltraSonic Sensor Pins - Mega Pin (RX_pin = 36, TX_pin = 37)
static const uint32_t GPS_BAUD = 9600;

static const int HORN_PIN = 5; //Pin for horn connected to relay - Mega Pin (42)

//**** Code provided by: http://www.instructables.com/id/Arduino-Powered-Autonomous-Vehicle/
//
#define MAX_DISTANCE_CM 600                        // Maximum distance we want to ping for (in CENTIMETERS). Maximum sensor distance is rated at 400-500cm.  
#define MAX_DISTANCE_IN (MAX_DISTANCE_CM / 2.5)    // same distance, in inches

#define FAST_SPEED 250 // 210 55
#define NORMAL_SPEED 250 // 190 75
#define TURN_SPEED 250 // 170 95
#define SLOW_SPEED 210 //150 125
#define NO_SPEED 127

int speed = NORMAL_SPEED;
int NOW_SPEED = speed;
#define REVERSE_SPEED 63 //50 170

#define TURN_LEFT 1
#define TURN_RIGHT 2
#define TURN_STRAIGHT 99

int targetHeading;              // where we want to go to reach current waypoint
int currentHeading;             // where we are actually facing now
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
#define HEADING_TOLERANCE 5     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

int sonarDistanceLeft = 0, sonarDistanceRight = 0;
// GPS Navigation

float currentLat, currentLong, targetLat, targetLong;
int distanceToTarget,            // current distance to target (current waypoint)
originalDistanceToTarget;    // distance to original waypoing when we started navigating to it


               // Waypoints
#define WAYPOINT_DIST_TOLERANE  1   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 5          // enter the numebr of way points here (will run from 0 to (n-1))
int waypointNumber = -1;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
waypointClass waypointList[NUMBER_WAYPOINTS] = { waypointClass(30.508302, -97.832624), waypointClass(30.508085, -97.832494), waypointClass(30.507715, -97.832357), waypointClass(30.508422, -97.832760), waypointClass(30.508518,-97.832665) };


// Steering/turning 
enum directions { left = TURN_LEFT, right = TURN_RIGHT, straight = TURN_STRAIGHT };
directions turnDirection = straight, oldTurnDirection = straight;


// Object avoidance distances (in inches)
#define SAFE_DISTANCE 90
#define TURN_DISTANCE 60
#define STOP_DISTANCE 40
//
// ****

// ****
// Code Provided by: 

static const int RC_NUM_CHANNELS = 2;

static const int RC_xAxis = 0;
static const int RC_yAxis = 1;

static const int RC_xAxis_INPUT = A5; // Pin for LinearActuator Mega - (A12)
static const int RC_yAxis_INPUT = A4; // Pin for Brushed DC Motor Mega - (A13)

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

//
// ****

// **** Code provided by: http://www.instructables.com/id/Rc-Controller-for-Better-Control-Over-Arduino-Proj/
//

#define TRC_NEUTRAL 127  // neutral speed
#define TRC_LEFT 240 // turn left
#define TRC_RIGHT 80 // turn right

static const int PWM_TURN = 9; //PWM signal pin for LinearAcutator (CH3)  Mega - (A12)
static const int PWM_SPEED = 12; //PWM signal pin for Brushed DC Motor (CH1) Mega - (A13)

//
// ****

TinyGPSPlus gps;

// Pins for AltSoftSerial
// TXPin = 46(9) RXPin = 48(8) UsuablePins = 44, 45 (10)
AltSoftSerial  altSerial;


void setup() {
  Serial.println("Starting");

  pinMode(RC_xAxis_INPUT, INPUT);
  pinMode(RC_yAxis_INPUT, INPUT);

  enableInterrupt(RC_xAxis_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_yAxis_INPUT, calc_ch3, CHANGE);

  pinMode(trig1_Pin, OUTPUT);
  pinMode(echo1_Pin, INPUT_PULLUP);
  pinMode(trig2_Pin, OUTPUT);
  pinMode(echo2_Pin, INPUT_PULLUP);
  
  pinMode(HORN_PIN, INPUT_PULLUP);
  pinMode(HORN_PIN, OUTPUT);

  pinMode(PWM_SPEED, OUTPUT);
  pinMode(PWM_TURN, OUTPUT);

  Serial.begin(SERIAL_PORT_BAUD);
  altSerial.begin(GPS_BAUD);

  Serial.println("Moving Forward\n");
  
  unsigned long start = millis();
  while( millis() - start < start_interval)
  {
    analogWrite(PWM_SPEED, FAST_SPEED);
    analogWrite(PWM_TURN, TRC_NEUTRAL);
  }
  Serial.println("\nSTART\n");
  Serial.println(F("Sats Current               Waypoint              New      Orginal  Sonar       Compass Speed           Degree               Direction  "));
  Serial.println(F("     Lat       Long        Lat        Long       Dist     Dist     Left  Right Heading Actual Internal Target Current Error            "));
  Serial.println(F("---------------------------------------------------------------------------------------------------------------------------------------"));


}

void loop() 
{
  autoHornController();
    if (auto_control == true)
      autoControl();
    else
    {
      rc_read_values();

      int RCx = rc_values[RC_xAxis];
      int RCy = rc_values[RC_yAxis];
  
      rc_control(RCx, RCy);
    }
    smartDelay(100);
}

static void autoControl()
{
  if (currentHeading != 0)
  {
  processGPS();
  currentHeading = readCompass();    // get our current heading
  smartDelay(0);
  }
  
  calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles      

  // distance in front of us, move, and avoid obstacles as necessary
  checkSonar();
  moveAndAvoid();
  updateDisplay();
  
  turnMotor();
  
  // update display and serial monitor    
  
   
}

void processGPS(void)
{

  currentLat = gps.location.lat();
  currentLong = gps.location.lng();
  
  // update the course and distance to waypoint based on our new position
  distanceToWaypoint();
  courseToWaypoint();

}   // processGPS(void)

void checkSonar(void)
{
  int distInLeft = getFrontLeftDistance();
  smartDelay(30);
  int distInRight = 120; //getFrontRightDistance();
  if (distInLeft > 235)                                // if too far to measure, return max distance;
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

void calcOppDesiredTurn(void)
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
    turnDirection = right; 
  }
  else if (headingError > 1)
  {
    turnDirection = left;
  }
  else
  {
    turnDirection = straight;
  }

}  // calcDesiredTurn()

void moveAndAvoid()
{
  if ((sonarDistanceLeft >= SAFE_DISTANCE) && (sonarDistanceRight >= SAFE_DISTANCE))       // no close objects in front of car
  {
    if (turnDirection == straight)
    {
      speed = FAST_SPEED; 
    }
    else
    {
      speed = NORMAL_SPEED;
    }
    setSpeed(speed);
  }

  if ((sonarDistanceLeft > TURN_DISTANCE) && (sonarDistanceLeft < SAFE_DISTANCE) || (sonarDistanceRight > TURN_DISTANCE) && (sonarDistanceRight < SAFE_DISTANCE))    // not yet time to turn, but slow down
  {
    if (turnDirection == straight)
      speed = NORMAL_SPEED;
    else
    {
      speed = TURN_SPEED;
    }
    setSpeed(speed);
  }

  if ((sonarDistanceLeft <  TURN_DISTANCE) || (sonarDistanceRight <  TURN_DISTANCE))  // getting close, time to turn to avoid object        
  {
    setSpeed(SLOW_SPEED);

    if ((LeftWall == true) && (RightWall == false))
    {
      turnDirection = right;
    }
    else if ((RightWall == true) && (LeftWall == false))
    {
      turnDirection = left;
    }
    else
    {
      if (headingError < -1)
      {
          turnDirection = left;
      }
      else if (headingError > 1)
      {
          turnDirection = right;
      }
    }
  }

  if ((sonarDistanceLeft < STOP_DISTANCE) && (sonarDistanceRight < STOP_DISTANCE))          // too close, stop and back up
  {
    setSpeed(NO_SPEED);
    turnDirection = straight;
    turnMotor();
    smartDelay(500);

    Serial.println("\n\nSTOPPING\n\n");

    while ((sonarDistanceLeft < TURN_DISTANCE) || (sonarDistanceRight < TURN_DISTANCE))       // backup until we get safe clearance
    {

      Serial.println("\n\nREVERSING\n\n");
       
      setSpeed(REVERSE_SPEED);
      
      autoHornController();
      
      processGPS();
      currentHeading = readCompass();    // get our current heading
      
      calcOppDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles
      checkSonar();   // distance in front of us, move, and avoid obstacles as necessary

      updateDisplay();    // update display and serial monitor

      turnMotor();
    }
    setSpeed(NO_SPEED);  // stop backing up
  }
} // end of IF TOO CLOSE

void nextWaypoint()
{
  Serial.println("\nGOING\n");
  waypointNumber++;
  targetLat = waypointList[waypointNumber].getLat();
  targetLong = waypointList[waypointNumber].getLong();

  if ((targetLat == 0 && targetLong == 0) || waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached? 
  {
     Serial.println("\nDONE\n");
    analogWrite(PWM_TURN, TRC_NEUTRAL);
    setSpeed(NO_SPEED);
    
    digitalWrite(HORN_PIN, HIGH);
    smartDelay(10);
    digitalWrite(HORN_PIN, LOW);
    smartDelay(100);
    digitalWrite(HORN_PIN, HIGH);
    smartDelay(10);
    digitalWrite(HORN_PIN, LOW);
    smartDelay(100);
    digitalWrite(HORN_PIN, HIGH);
    smartDelay(10);
    digitalWrite(HORN_PIN, LOW);
    smartDelay(1000);

     waypointNumber = 0;

    waypointNumber++;
  targetLat = waypointList[waypointNumber].getLat();
  targetLong = waypointList[waypointNumber].getLong();
    //loopForever();
  }

  processGPS();
  distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
  courseToWaypoint();

}  // nextWaypoint()

void loopForever(void)
{
  while (1)
    ;
}

void updateDisplay(void)
{
  static unsigned long lastUpdate = millis();       // for controlling frequency of LCD updates
  unsigned long currentTime;

  // check time since last update
  currentTime = millis();
  if (lastUpdate > currentTime)   // check for time wrap around
    lastUpdate = currentTime;

  

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(currentLat, gps.location.isValid(), 11, 6);
  printFloat(currentLong, gps.location.isValid(), 11, 6);
  printFloat(waypointList[waypointNumber].getLat(), true, 11, 6);
  printFloat(waypointList[waypointNumber].getLong(), true, 11, 6);
  printInt(distanceToWaypoint(),gps.location.isValid(), 9);
  printInt(originalDistanceToTarget, gps.location.isValid(), 9);
 
  
  printInt(sonarDistanceLeft, true, 6);
  printInt(sonarDistanceRight, true, 6);
  
  printFloat(currentHeading, gps.course.isValid(), 8, 2);
  printFloat(gps.speed.mph(), gps.speed.isValid(), 7, 2);
  printFloat(speed, true, 9, 2);
  printFloat(targetHeading, gps.location.isValid(), 7, 2);
  printFloat(currentHeading, gps.course.isValid(), 8, 2);
  printInt(headingError, true, 6);
  
  /*Serial.print(F("Free Memory: "));
  Serial.println(freeRam(), DEC);*/
  
}  // updateDisplay()  

int freeRam()   // display free memory (SRAM)
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
} // freeRam()

//
// ****

void turnNeutral()
{
  Serial.println("Straight");
  analogWrite(PWM_TURN, TRC_NEUTRAL);
}

void turnLeft()  //turn to the left
{
  Serial.println("Left");
  analogWrite(PWM_TURN, TRC_LEFT);
}

void turnRight() //turn to the right
{
  Serial.println("Right");
  analogWrite(PWM_TURN, TRC_RIGHT);
}

void setSpeed(int speed)
{
  analogWrite(PWM_SPEED, speed);
}

void turnMotor()
{
  if (turnDirection == left)
  {
      turnLeft();
  }
  else if (turnDirection == right)
  { 
      turnRight();
  }
  else 
  {
      turnNeutral();
  }
}


static void autoHornController()
{
  if (hornInUse == false)
  {
    hornInUse = true;
    digitalWrite(HORN_PIN, LOW);
  
    unsigned long start = millis();
  
    unsigned long in_start = 0;
    
    if (start - startTime_1 >= long_interval)
    {
      Serial.println("\nHORN\n");
      digitalWrite(HORN_PIN, HIGH);
      smartDelay(100);
      digitalWrite(HORN_PIN, LOW);
  
      startTime_1 = start;
    }
    hornInUse = false;
  }
}

void hornController()
{
  if (hornInUse == false)
  {
    hornInUse = true;
    digitalWrite(HORN_PIN, LOW);
  
    unsigned long start_2 = millis();
    
    if (start_2 - startTime_2 >= short_interval)
    {
      Serial.println("\nHORN\n");
      digitalWrite(HORN_PIN, HIGH);
      smartDelay(100);
      digitalWrite(HORN_PIN, LOW);
  
      startTime_2 = start_2;
    }
    hornInUse = false;
  }
}

static int getFrontLeftDistance()
{
  long duration = 0;
  int distance = 0;

  // Clears the trigPin
  digitalWrite(trig1_Pin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig1_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1_Pin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  // Version 2.0 requires echo pin to be pulled up to VCC. 
  // A 4.7K  to 10K resistor can be used as pull-up resistor. (Uses 10k)
  duration = pulseIn(echo1_Pin, HIGH);

  Serial.print(" Duration Left: ");
  Serial.println(duration);

  if (duration < 1740)
  {
    duration = 1740;
  }

  // Calculating the distance (cm)
  distance = (int)(duration / 74 / 2);

  if (distance < TURN_DISTANCE)
  {
    LeftWall = true;
  }
  else
  {
    LeftWall = false;
  }

  if (duration >= 20)
  {
    if ((distance <= TURN_DISTANCE) && (hornInUse == false))
    {
      hornController();
    }
    return distance;
  }
  
  smartDelay(50);
}

//not used temp.
static int getFrontRightDistance()
{
  long duration = 0;
    
  int distance = 0;

  // Clears the trigPin
  digitalWrite(trig2_Pin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig2_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2_Pin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  // Version 2.0 requires echo pin to be pulled up to VCC. 
  // A 4.7K  to 10K resistor can be used as pull-up resistor. (Uses 10k)
  duration = pulseIn(echo2_Pin, HIGH);

  if (duration < 1680)
  {
    duration = 1680;
  }

  // Calculating the distance (in)
  distance = (int)(duration / 74 / 2);

  if (distance <= TURN_DISTANCE)
  {
    RightWall = true;
  }
  else
  {
    RightWall = false;
  }

 if (duration >= 20)
  {
    if ((distance <= TURN_DISTANCE) && (hornInUse == false))
    {
      hornController();
    }
    return distance;
  }
  
  smartDelay(50);
}

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


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (altSerial.available())
      gps.encode(altSerial.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}
