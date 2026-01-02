# wavypoint-robo
 #include <Wire.h>

#include <Adafruit_QMC5883P.h>

#include <TinyGPSPlus.h>

#include <SoftwareSerial.h>

#include <PRIZM.h>

#include <math.h>

// ==========================================

// === FINAL CALIBRATED OFFSET ===

// ==========================================

const float COMPASS_OFFSET = 145;

const bool PURE_COMPASS_MODE = true;

// ==========================================

// === Hardware ===

#define LEFT_MOTOR 1

#define RIGHT_MOTOR 2

SoftwareSerial BT(2, 9);

SoftwareSerial GPS_Serial(3, 4);

// === Objects ===

Adafruit_QMC5883P qmc;

TinyGPSPlus gps;

PRIZM prizm;

// === Constants ===

float declinationDeg = -0.6;

const float WHEEL_RADIUS = 0.045;

const float WHEEL_BASE = 0.34;

const float TICKS_PER_REV = 1440.0;

const bool INVERT_COMPASS = false;

// === Speed Settings ===

const int CRUISE_SPEED = 50;

const int TURN_SPEED = 45;

const int MIN_MOTOR_POWER = 25;

// === Waypoints ===

struct NavPoint { float lat; float lon; };

NavPoint waypoints[] = {

{17.780223,83.375458},

{17.780277, 83.375477},

{17.780322, 83.375368},

{17.780305,83.375359}

};

int totalWaypoints = sizeof(waypoints) / sizeof(waypoints[0]);

int currentWaypoint = 0;

bool navigationActive = true;

// === Internal Variables ===

float trustedHeading = 0;

unsigned long lastCompassTime = 0;

const unsigned long COMPASS_INTERVAL = 100;

long lastLeftCount = 0;

long lastRightCount = 0;

float encoderHeadingDelta = 0;

float smoothLat = 0, smoothLon = 0;

const float GPS_ALPHA = 0.3;

int16_t offsetX = 0, offsetY = 0;

// ==========================================

// === GPS 5Hz CONFIGURATION ===

// ==========================================

const unsigned char UBLOX_INIT[] PROGMEM = {

// Disable NMEA GSV (Satellites in view - uses too much bandwidth)

0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38,

// Disable NMEA GLL (Redundant)

0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A,

// Disable NMEA GSA (Redundant)

0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x31,

// Disable NMEA VTG (Redundant)

0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x46,

// Set Rate to 5Hz (200ms) - This is the key fix for overshoot!

0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A

};

// === Safe Power Function ===

void setSafeMotorPower(int left, int right) {

if (left > 0 && left < MIN_MOTOR_POWER) left = MIN_MOTOR_POWER;

if (left < 0 && left > -MIN_MOTOR_POWER) left = -MIN_MOTOR_POWER;

if (right > 0 && right < MIN_MOTOR_POWER) right = MIN_MOTOR_POWER;

if (right < 0 && right > -MIN_MOTOR_POWER) right = -MIN_MOTOR_POWER;

prizm.setMotorPower(LEFT_MOTOR, left);

prizm.setMotorPower(RIGHT_MOTOR, right);

}

void setup() {

Serial.begin(115200);

BT.begin(9600);

GPS_Serial.begin(9600);

delay(100);

// Send the configuration commands to the GPS

for(unsigned int i = 0; i < sizeof(UBLOX_INIT); i++) {

GPS_Serial.write(pgm_read_byte(UBLOX_INIT + i));

delay(5); // Wait 5ms between bytes so the GPS doesn't choke

}

delay(500); // Wait for the GPS to process the changes

prizm.PrizmBegin();

prizm.setMotorInvert(RIGHT_MOTOR, 1);

if (!qmc.begin()) {

while (1) { BT.println("Compass Fail"); delay(1000); }

}

qmc.setMode(QMC5883P_MODE_CONTINUOUS);

qmc.setODR(QMC5883P_ODR_50HZ);

qmc.setOSR(QMC5883P_OSR_8);

qmc.setRange(QMC5883P_RANGE_12G);

calibrateCompass();

trustedHeading = getCompassHeading();

BT.println("Ready. Waiting for GPS...");

}

void loop() {

if (BT.available()) {

char c = BT.read();

if (c == 'S' || c == 's') { navigationActive = false; stopMotors(); }

}

if (!navigationActive) return;

while (GPS_Serial.available()) gps.encode(GPS_Serial.read());

updateEncoderDelta();

if (gps.location.isUpdated()) {

float rawLat = gps.location.lat();

float rawLon = gps.location.lng();



if (smoothLat == 0) { smoothLat = rawLat; smoothLon = rawLon; }

smoothLat += GPS_ALPHA * (rawLat - smoothLat);

smoothLon += GPS_ALPHA * (rawLon - smoothLon);



NavPoint target = waypoints[currentWaypoint];

float distToTarget = calculateDistance(smoothLat, smoothLon, target.lat, target.lon);

float bearingToTarget = calculateBearing(smoothLat, smoothLon, target.lat, target.lon);



if (millis() - lastCompassTime > COMPASS_INTERVAL) {

  trustedHeading = getCompassHeading();

  encoderHeadingDelta = 0;

  lastCompassTime = millis();

}



float currentHeading = PURE_COMPASS_MODE ? trustedHeading : getHybridHeading();

float headingError = getShortestRotation(bearingToTarget, currentHeading);



BT.print("Hdg:"); BT.print(currentHeading, 0);

BT.print(" Tgt:"); BT.print(bearingToTarget, 0);

BT.print(" Err:"); BT.print(headingError, 0);

BT.print(" Dist:"); BT.println(distToTarget, 1);



if (distToTarget < 5.0) {

  stopMotors();

  BT.print("Reached WP "); BT.println(currentWaypoint);

  delay(2000);

  currentWaypoint++;

  if (currentWaypoint >= totalWaypoints) {

    BT.println("Done!");

    navigationActive = false;

  }

} else {

  executeMovement(headingError, distToTarget);

}

}

}

void executeMovement(float headingError, float dist) {

if (abs(headingError) > 25.0) {

int pwr = TURN_SPEED;

if (headingError > 0) {

  setSafeMotorPower(pwr, -pwr);

} else {

  setSafeMotorPower(-pwr, pwr);

}

} else {

float Kp = 2.0;

int correction = (int)(headingError * Kp);



int leftPwr = CRUISE_SPEED + correction;

int rightPwr = CRUISE_SPEED - correction;



leftPwr = constrain(leftPwr, 0, 100);

rightPwr = constrain(rightPwr, 0, 100);



setSafeMotorPower(leftPwr, rightPwr);

}

}

float getCompassHeading() {

int16_t x, y, z;

if (qmc.isDataReady() && qmc.getRawMagnetic(&x, &y, &z)) {

float calX = (float)(x - offsetX);

float calY = (float)(y - offsetY);



float heading = atan2(calY, calX) * 180.0 / PI;



if (INVERT_COMPASS) heading = -heading;



heading += declinationDeg;

heading += COMPASS_OFFSET;



heading += 180;  // sensor backwards

heading -= 90;   // sensor rotated sideways → FIX WEST instead of SOUTH



while (heading < 0) heading += 360;

while (heading >= 360) heading -= 360;



return heading;

}

return trustedHeading;

}

void calibrateCompass() {

BT.println("CALIBRATING: Spin Robot 360 deg NOW!");

int16_t minX = 32000, maxX = -32000, minY = 32000, maxY = -32000;

int16_t x, y, z;

unsigned long start = millis();

while (millis() - start < 8000) {

if (qmc.isDataReady() && qmc.getRawMagnetic(&x, &y, &z)) {

  if (x < minX) minX = x; if (x > maxX) maxX = x;

  if (y < minY) minY = y; if (y > maxY) maxY = y;

}

}

offsetX = (minX + maxX) / 2;

offsetY = (minY + maxY) / 2;

BT.println("Calibration Done.");

}

float getHybridHeading() {

float h = trustedHeading + encoderHeadingDelta;

while (h < 0) h += 360;

while (h >= 360) h -= 360;

return h;

}

void updateEncoderDelta() {

if (PURE_COMPASS_MODE) return;

long l = prizm.readEncoderCount(LEFT_MOTOR);

long r = prizm.readEncoderCount(RIGHT_MOTOR);

long dL = l - lastLeftCount;

long dR = r - lastRightCount;

lastLeftCount = l;

lastRightCount = r;

float distL = (2 * PI * WHEEL_RADIUS) * (dL / TICKS_PER_REV);

float distR = (2 * PI * WHEEL_RADIUS) * (dR / TICKS_PER_REV);

float thetaChangeRad = (distL - distR) / WHEEL_BASE;

encoderHeadingDelta += (thetaChangeRad * 180.0 / PI);

}

void stopMotors() {

prizm.setMotorPower(LEFT_MOTOR, 0);

prizm.setMotorPower(RIGHT_MOTOR, 0);

}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {

float R = 6371000;

float dLat = radians(lat2 - lat1);

float dLon = radians(lon2 - lon1);

float a =

sin(dLat / 2) * sin(dLat / 2) +

cos(radians(lat1)) * cos(radians(lat2)) *

sin(dLon / 2) * sin(dLon / 2);

return R * 2 * atan2(sqrt(a), sqrt(1 - a));

}

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {

float dLon = radians(lon2 - lon1);

float y = sin(dLon) * cos(radians(lat2));

float x =

cos(radians(lat1)) * sin(radians(lat2)) -

sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);

float brng = atan2(y, x) * 180.0 / PI;

if (brng < 0) brng += 360;

return brng;

}

float getShortestRotation(float target, float current) {

float diff = target - current;

if (diff > 180) diff -= 360;

if (diff < -180) diff += 360;

return diff;

}
