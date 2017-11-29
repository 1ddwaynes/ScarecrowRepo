#include <TinyGPS++.h>
#include <AltSoftSerial.h>
#include <EnableInterrupt.h>

static const uint32_t SERIAL_PORT_BAUD = 115200;
static const int RC_NUM_CHANNELS = 4;

static const int RC_xAxis = 0;
static const int RC_yAxis = 1;

static const int RC_xAxis_INPUT = A12; //A12
static const int RC_yAxis_INPUT = A13; //A13

unsigned long startTime;
unsigned long interval = 30000;


static const int enA = 9, enB = 10;
static const int triGPS_RX_Pin = 34, echo_TX_Pin = 35;
static const uint32_t GPS_BAUD = 9600;
static const int LED_G_Pin = 5, HORN_PIN = 42, ONOFF_PIN = 43; //motor_PIN = 4;
static const int opSens = A3;

int signalAInput1;                             // signal input 1 for encoderA
int signalAInput2;                             // signal input 2 for encoderA
int signalBInput1;                             // signal input 1 for encoderB
int signalBInput2;                             // signal input 2 for encoderB



											   // robot location service
											   // 0 is up
											   // 1 is right
											   // 2 is down
											   // 3 is left
int robotDirection = 2;

// this is the coordinates in the grid of where the robot is
// it is also the x and y indexes in the array.
// remember that the array starts at index 0.
int xcoordinate = 2;
int ycoordinate = 1;

						  // //motor pins
//const int motor1Pin = 9;  // Left side
//const int motor2Pin = 10; // right side

						  // the array that it tracks with
						  // this can be an array of any size
						  // just make sure that the robot has a free space to move to from its initial position.
int arraything[2][2] =
{
	{ 1,1 }
	,
	{ 1,1 }
};

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

TinyGPSPlus gps;

// Pins for AltSoftSerial
// TXPin = 46(7) RXPin = 48(8) UsuablePins = 44, 45
AltSoftSerial  altSerial; 

unsigned long last = 0UL;

void setup() {

  Serial.println("Starting");
  // put your setup code here, to run once:
  pinMode(triGPS_RX_Pin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echo_TX_Pin, INPUT); // Sets the echoPin as an Input
  pinMode(RC_xAxis_INPUT, INPUT);
  pinMode(RC_yAxis_INPUT, INPUT);
  pinMode(opSens, INPUT);

  // Rotary encoder
  //pinMode(encoderAPin, INPUT);
  //pinMode(encoderBPin, INPUT);

  pinMode(LED_G_Pin, OUTPUT);

  enableInterrupt(RC_xAxis_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_yAxis_INPUT, calc_ch2, CHANGE);
  pinMode(HORN_PIN,INPUT_PULLUP);
  pinMode(ONOFF_PIN,INPUT_PULLUP);
  pinMode(HORN_PIN,OUTPUT);
  pinMode(ONOFF_PIN,OUTPUT);

  Serial.begin(SERIAL_PORT_BAUD);
  altSerial.begin(GPS_BAUD);

 //Serial.write("Debug mode");
}

void loop() {
//  int value_op = analogRead(opSens);
//  Serial.println(value_op);
  analogWrite(LED_G_Pin, 20);
  
  while (altSerial.available()>0)
    gps.encode(altSerial.read());

 //   debug();
    
    Serial.print(F("Sat"));
    printInt(gps.satellites.value(), gps.satellites.isValid(), 2);
    
    Serial.print(F("Lat"));
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    
    Serial.print(F("Lon"));
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);

    Serial.print(F("Spd"));
    printFloat(gps.speed.mph(), gps.speed.isValid(), 6, 2);

    Serial.print(F("Dst"));
    printInt(getFrontDistance(),true, 6);
    //float distance = getFrontDistance();
    //horn_controller(distance);

	auto_horn_controller();
    
    rc_read_values();

    Serial.print(F("yAX")); Serial.print(rc_values[RC_xAxis]); Serial.println("\t");
    Serial.print(F("xAX")); Serial.print(rc_values[RC_yAxis]); Serial.println("\t");

    int RC1 = rc_values[RC_xAxis];
    int RC2 = rc_values[RC_yAxis];

    rc_control(RC1, RC2);
    
// no less than 50 ms (as per JSN Ultrasonic sensor specification)
smartDelay(60);
}

void calc_ch1() { calc_input(RC_xAxis, RC_xAxis_INPUT); }
void calc_ch2() { calc_input(RC_yAxis, RC_yAxis_INPUT); }

void debug()
{
    if (millis() > 5000 && gps.charsProcessed() < 10) // uh oh
    {
        Serial.println("ERROR: not getting any GPS data!");
        // dump the stream to Serial
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

static void auto_horn_controller()
{
	unsigned long start = millis();

	if (start - startTime >= interval) {
		digitalWrite(HORN_PIN, HIGH);
		smartDelay(100);
		digitalWrite(HORN_PIN, LOW);
		startTime = millis();
	}
}
void horn_controller(float distance)
{
  if ( distance < 60 && distance > 0) {
     digitalWrite(HORN_PIN, HIGH);
     smartDelay(500);
     digitalWrite(HORN_PIN, LOW);
  }
  else
  {
    digitalWrite(HORN_PIN, LOW);
  }
}

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
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
			horn_controller(distanceCm);
		}
		return distanceCm;
	}
}

static void rc_control(int xAxis, int yAxis)
{
	int linearActuator = map(yAxis, 1828, 1160, 0, 255); // Map the potentiometer value from 0 to 255
	analogWrite(enA, linearActuator); // Send PWM signal to L298N Enable pin
	Serial.println("1");
	Serial.println(linearActuator);
	int DCmotor = map(xAxis, 1932, 1088, 0, 255); // Map the potentiometer value from 0 to 255
	Serial.println("2");
	Serial.println(DCmotor);
	analogWrite(enB, DCmotor); // Send PWM signal to L298N Enable pin
}

static void auto_control(double rightDetect, double leftDetect)
{
	if (rightDetect > 200 && leftDetect > 200)
	{
		//move  forward
	}

	//stop foward
	//cant use back and foward motors at the same time

	else if (rightDetect <= 180 || leftDetect <= 180 )
	{ 
		if (rightDetect > leftDetect)
		{
			//turn to right
			//limit the amount of the right turning
		}

		else
		{
			//move to left
		}
	}

	else if (rightDetect == leftDetect)
	{
		//move back some distance
		//stop back
		//turn left
		//move forward
	}
}

static int alighmToGeoCompass()
{
	//if GPS direction is not same GPS location direction
	{
		//if currently North 
		{
			//if des cord > 0 && cur cord > 0
			{
				//des cord - curr cord
			}
			//else if des cord < 0 && cur cord < 0
			{
				//print not coded for southern hemisphere
			}

			//if lat cord != 
			//stop forward
			//turn right
		}
		//if North
	}
}


//
// code provided by http://www.instructables.com/id/Autonomous-Autonavigation-Robot-Arduino/
//
static void control()
{
	if (isFrontOpen() == true)
	{
		moveForward();
		smartDelay(2000);
	}
	else if (isRightOpen() == true)
		{
			turnRight();
			smartDelay(2000);
		}
	else if (isLeftOpen() == true)
	{
		turnLeft();
		smartDelay(2000);
	}
	else
	{
		turnAround();
		smartDelay(2000);
	}
}

// Checks if there is something right in front of it using Grids
static boolean isFrontOpen() {
	int nextNumber = getFrontNumber();
	if (nextNumber == 0) {
		return true;
	}
	else {
		return false;
	}
}

// Checks if there is something to the Right of it using Grids
static boolean isRightOpen() {
	int nextNumber = getRightNumber();
	if (nextNumber == 0) {
		return true;
	}
	else {
		return false;
	}
}

// Checks if there is something to the Left of it using Grids
static boolean isLeftOpen() {
	int nextNumber = getLeftNumber();
	if (nextNumber == 0) {
		return true;
	}
	else {
		return false;
	}
}

// Moves straight forward.
static void moveForward() {
	////motor1.write(180);
	////motor2.write(0);

	Serial.println("Forward");
	if (robotDirection == 0)
		ycoordinate = ycoordinate - 1;
	if (robotDirection == 1)
		xcoordinate = xcoordinate + 1;
	if (robotDirection == 2)
		ycoordinate = ycoordinate + 1;
	if (robotDirection == 3)
		xcoordinate = xcoordinate - 1;
	smartDelay(100);
	/*Serial.print("  xcoordinate " );
	Serial.print(xcoordinate);
	delay (500);
	Serial.print(" ycoordinate ");
	Serial.print(ycoordinate);
	delay (500);
	Serial.print("  robot direction: ");
	Serial.print(robotDirection);
	delay(500);
	Serial.println ();
	delay(1000);

	*/
	smartDelay(800);
}

// Turns 90 degrees to the Right
static void turnRight() {
	////motor1.write(60);
	////motor2.write(60);
	smartDelay(178);
	////motor2.write(95);
	smartDelay(65);
	////motor1.write(90);
	Serial.println("Right");
	if (robotDirection == 0)
		robotDirection = 1;
	else if (robotDirection == 1)
		robotDirection = 2;
	else if (robotDirection == 2)
		robotDirection = 3;
	else if (robotDirection == 3)
		robotDirection = 0;
	smartDelay(500);
	Serial.print("  xcoordinate ");
	Serial.print(xcoordinate);
	smartDelay(500);
	Serial.print(" ycoordinate ");
	Serial.print(ycoordinate);
	smartDelay(500);
	Serial.print("  robot direction: ");
	Serial.print(robotDirection);
	smartDelay(500);
	Serial.println();

	smartDelay(1000);
}

// Turns 90 degrees to the Left
static void turnLeft() {
	//motor1.write(120);
	//motor2.write(120);
	smartDelay(325);
	//motor2.write(95);
	smartDelay(65);
	//motor1.write(90);
	Serial.println("Left");
	if (robotDirection == 0)
		robotDirection = 3;
	else if (robotDirection == 1)
		robotDirection = 0;
	else if (robotDirection == 2)
		robotDirection = 1;
	else if (robotDirection == 3)
		robotDirection = 2;
	smartDelay(500);
	Serial.print("  xcoordinate ");
	Serial.print(xcoordinate);
	smartDelay(500);
	Serial.print(" ycoordinate ");
	Serial.print(ycoordinate);
	smartDelay(500);
	Serial.print("  robot direction: ");
	Serial.print(robotDirection);
	smartDelay(500);
	Serial.println();
	smartDelay(1000);
}

// Turns 180 degrees
static void turnAround() {
	//  delay(1000);
	Serial.println("Around");
	if (robotDirection == 0)
		robotDirection = 2;
	else if (robotDirection == 1)
		robotDirection = 3;
	else if (robotDirection == 2)
		robotDirection = 0;
	else if (robotDirection == 3)
		robotDirection = 1;
	smartDelay(500);
	Serial.print("  xcoordinate ");
	Serial.print(xcoordinate);
	smartDelay(500);
	Serial.print(" ycoordinate ");
	Serial.print(ycoordinate);
	smartDelay(500);
	Serial.print("  robot direction: ");
	Serial.print(robotDirection);

	smartDelay(1000);
}

// Gets the number on the Grid of the space right in front of it.
static int getFrontNumber() {
	if (robotDirection == 0) {
		return arraything[ycoordinate - 1][xcoordinate];
	}
	if (robotDirection == 1) {
		return arraything[ycoordinate][xcoordinate + 1];
	}
	if (robotDirection == 2) {
		return arraything[ycoordinate + 1][xcoordinate];
	}
	if (robotDirection == 3) {
		return arraything[ycoordinate][xcoordinate - 1];
	}
}

// Gets the number on the Grid of the space to the Right of it.
static int getRightNumber() {
	if (robotDirection == 0) {
		return arraything[ycoordinate][xcoordinate + 1];

	}
	if (robotDirection == 1) {
		return arraything[ycoordinate + 1][xcoordinate];

	}
	if (robotDirection == 2) {
		return arraything[ycoordinate][xcoordinate - 1];
	}
	if (robotDirection == 3) {
		return arraything[ycoordinate - 1][xcoordinate];
	}
}

// Gets the number on the Grid of the Space to the Left of it.
static int getLeftNumber() {
	if (robotDirection == 0) {
		return arraything[ycoordinate][xcoordinate - 1];
	}
	if (robotDirection == 1) {
		return arraything[ycoordinate - 1][xcoordinate];
	}
	if (robotDirection == 2) {
		return arraything[ycoordinate][xcoordinate + 1];
	}
	if (robotDirection == 3) {
		return arraything[ycoordinate + 1][xcoordinate];
	}
}
