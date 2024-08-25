#define DEBUG
#include "Globals.h"
#include "debug.h"
#include "SM_16UNIVIN.h"
#include "SM_RTD.h"  // Temp sensors
#include "SM_16DIGIN.h"
#include "Adafruit_FRAM_I2C.h"
#include "Adafruit_MCP9601.h"
#include "Adafruit_MPU6050.h"
#include "AuberinsSensors.h"  // Pressure sensors
#include "CruiseControl.h"

int dig_indicator[] = { DIG_OVER_DRIVE, DIG_TCC, DIG_LEFT, DIG_RIGHT, DIG_BRAKE, DIG_HEAD_LOW, DIG_HEAD_HIGH, DIG_RUNNING, DIG_WATER_FUEL, DIG_LOW_WASHER};

// defined by cruise state
unsigned int cruiseAccel = 0;
unsigned int cruiseActive = 0;
unsigned int cruiseDecel = 0;
unsigned int cruiseSetValue = 0;
unsigned int cruiseSpeedActive = 0;

// define digitalPins
unsigned int digitalPins = 0;

// Used to calculate Speed
volatile unsigned long lastTimeVSS = 0;
volatile unsigned long periodVSS = 0;
volatile unsigned long lastTimeTACH = 0;
volatile unsigned long periodTACH = 0;

// Odometer
int totalMiles = 0;           // in number of 1/10 of mile
int odometer = 0;             // Current odometer reading in 1/10 of a mile
int accumulatedDistance = 0;  // Accumulated distance since last odometer update

const int vssPulsesPerRevolution = 48;      // Number of pulses per wheel revolution
const int tachPulsesPerRevolution = 48;      // Number of pulses per wheel revolution
const float wheelDiameterInches = 20.0;  // Wheel diameter in inches (example value)

int speed = 0;
int rpm = 0;
int gearPosition = 0;

// Temp
int EGTemp = 0;
int iaTemp = 0;
int oilTemp = 0;
int coolantTemp = 0;
int transTemp = 0;
int ambientTemp = 0;

// Pressure 100 * float(pressure) need to divide by 100 in custom.xml to get pressure
int oilPressure = 0;
int fuelPressure = 0;
int boostPressure = 0;

//MPU6050
int accelerationX = 0;
int accelerationY = 0;
int accelerationZ = 0;

unsigned long previousMillis = 0;  // Stores the last time the procedure was called

//FRAM Memory Map
// Odometer: 0x0

// Create objects
SM_16_UNIVIN adc_card(0, &Wire1);  // Sixteen Universal Inputs HAT with stack level 0 (no jumpers installed)
SM_16DIGIN dig_card(1, &Wire1);  // Sixteen Universal Inputs HAT with stack level 1
SM_RTD rtd_card(2, &Wire1);        // 8 RTD Inputs HAT with stack level 2

Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();  // Create FRAM object
Adafruit_MCP9601 mcp;
Adafruit_MPU6050 mpu;

void setup() {
  // put your setup code here, to run once:
  pinMode(PWM_SPEED, INPUT);
  pinMode(PWM_TACH, INPUT);

  // attachInterrupt(digitalPinToInterrupt(PWM_SPEED), measurePeriodVSS, RISING);
  // attachInterrupt(digitalPinToInterrupt(PWM_TACH), measurePeriodTACH, RISING);

  Serial1.begin(115200);
  delay(1000);
  Serial.begin(115200);
  delay(1000);

  // Initialize Sequent cards
  if (adc_card.begin()) {
    DB_PRINT("ADC Sixteen Universal Inputs Card detected\n");
  } else {
    DB_PRINT("ADC Sixteen Universal Inputs Card NOT detected!\n");
  }

  if (dig_card.begin()) {
    DB_PRINT("DIG Sixteen Universal Inputs Card detected\n");
  } else {
    DB_PRINT("DIG Sixteen Universal Inputs Card NOT detected!\n");
  }

  if (rtd_card.begin()) {
    DB_PRINT("RTD Inputs Card detected\n");
  } else {
    DB_PRINT("RTD Inputs Card NOT detected!\n");
  }

  // Initialize FRAM and read odometer value
  if (fram.begin(I2C_ODOMETER_ADR, &Wire1)) {
    DB_PRINTLN("Found I2C FRAM");
    // Read the initial odometer value from FRAM
    fram.read(0, (uint8_t*)&odometer, sizeof(odometer));
    DB_PRINT("Initial Odometer Reading: ");
    DB_PRINTLN(odometer);
  } else {
    DB_PRINTLN("I2C FRAM not identified ... check your connections?");
    // while (1);
  }

  // Initialize EGT reader
  if (!mcp.begin(I2C_EGT_ADR, &Wire1)) {
    DB_PRINTLN("Failed to find MCP9600 chip");
    // while (1) { delay(10); }
  } else {
    DB_PRINTLN("MCP9600 Found!");
    // Set the thermocouple type to K
    mcp.setThermocoupleType(MCP9600_TYPE_K);
    DB_PRINT("Thermocouple type: ");
    switch (mcp.getThermocoupleType()) {
      case MCP9600_TYPE_K: DB_PRINTLN("K"); break;
      default: DB_PRINTLN("Unknown"); break;
    }
  }

  // Initialize accelerometer
  if (!mpu.begin(I2C_GPS_ADR, &Wire1)) {
    DB_PRINTLN("Failed to find MPU6050 chip");
    // while (1) { delay(10); }
  } else {
    DB_PRINTLN(" MPU6050 Found");

    //setupt motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 250) {
    unsigned long drivingTimeAtSpeed = currentMillis - previousMillis;
    previousMillis = currentMillis;

    readDigital();
    cruise();  // Call the procedure

    // speed = calculateSpeed();
    // updateMilesDriven(speed, drivingTimeAtSpeed);
    // rpm = calculateRPM();

    // readEGTemp();
    // calculatePressures();
    // calculateTemps();

    if (mpu.getMotionInterruptStatus()) {
      /* Get new sensor events with the readings */
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      accelerationX = int(a.acceleration.x * INT_SCALING);
      accelerationY = int(a.acceleration.y * INT_SCALING);
      accelerationZ = int(a.acceleration.z * INT_SCALING);

      /* Print out the values */
      DB_PRINT("AccelX:");
      DB_PRINT(accelerationX);
      DB_PRINT(",");
      DB_PRINT("AccelY:");
      DB_PRINT(accelerationY);
      DB_PRINT(",");
      DB_PRINT("AccelZ:");
      DB_PRINT(accelerationZ);
      DB_PRINT(", ");
      DB_PRINT("GyroX:");
      DB_PRINT(g.gyro.x);
      DB_PRINT(",");
      DB_PRINT("GyroY:");
      DB_PRINT(g.gyro.y);
      DB_PRINT(",");
      DB_PRINT("GyroZ:");
      DB_PRINT(g.gyro.z);
      DB_PRINTLN("");
    }

    SendCANFramesToSerial();
  }
}

void calculatePressures() {
  oilPressure = calculatePressure5BAR(adc_card.readAnalogMv(ADC_OIL_PRESSURE));
  fuelPressure = calculatePressure5BAR(adc_card.readAnalogMv(ADC_FUEL_PRESSURE));
  boostPressure = calculatePressure5BAR(adc_card.readAnalogMv(ADC_BOOST_PRESSURE));
  DB_PRINT("oilP: ");
  DB_PRINTLN(oilPressure);
  DB_PRINT("fuelP: ");
  DB_PRINTLN(fuelPressure);
  DB_PRINT("boostP: ");
  DB_PRINTLN(boostPressure);
  }

void calculateTemps() {
  iaTemp = int(rtd_card.readTemp(RTD_IA_TEMP) * INT_SCALING);
  oilTemp = int(rtd_card.readTemp(RTD_OIL_TEMP) * INT_SCALING);
  coolantTemp = int(rtd_card.readTemp(RTD_COOLANT_TEMP) * INT_SCALING);
  transTemp = int(rtd_card.readTemp(RTD_TRANS_TEMP) * INT_SCALING);
  ambientTemp = int(rtd_card.readTemp(RTD_AMBIENT_TEMP) * INT_SCALING);
  DB_PRINT("iaT: ");
  DB_PRINTLN(iaTemp);
  DB_PRINT("oilT: ");
  DB_PRINTLN(oilTemp);
  DB_PRINT("coolantT: ");
  DB_PRINTLN(coolantTemp);
  DB_PRINT("transT: ");
  DB_PRINTLN(transTemp);
  DB_PRINT("ambientT:");
  DB_PRINTLN(ambientTemp);
}

void readEGTemp() {
  // Read the temperature in Celsius
  EGTemp = int(mcp.readThermocouple() * INT_SCALING);
  DB_PRINT("EGT: ");
  DB_PRINTLN(EGTemp);
}

void updateMilesDriven(float speed, unsigned long timeMillis) {
  float timeHours = timeMillis / 3600000.0;  // Convert time from milliseconds to hours
  int distance = int (speed * timeHours * 10);        // Calculate distance as the number of 1/10 of a mile
  totalMiles += distance;                    // Update total tenths of a miles driven

  accumulatedDistance += distance;  // Accumulate distance

  if (accumulatedDistance >= 1) {   // Check if accumulated distance is 1/10 of a mile or more
    odometer += accumulatedDistance;  // Update odometer
    accumulatedDistance = 0;        // Reset accumulated distance
  }
  // Write the updated odometer value to FRAM
  // fram.write(0, (uint8_t*)&odometer, sizeof(odometer));
  DB_PRINT("ODO: ");
  DB_PRINTLN(odometer);
  DB_PRINT("TRIP: ");
  DB_PRINTLN(accumulatedDistance);
}

void measurePeriodVSS() {
  unsigned long currentTime = millis();
  periodVSS = currentTime - lastTimeVSS;
  lastTimeVSS = currentTime;
}

float calculateSpeed() {
  noInterrupts();
  unsigned long periodMillis = periodVSS;
  interrupts();

  // DB_PRINTLN(periodMillis);

  if (periodMillis > 0) {
    // Convert period to seconds
    float periodSeconds = (periodMillis * vssPulsesPerRevolution) / (1000.0);

    // Calculate circumference (in inches)
    float circumferenceInches = 2 * PI * wheelDiameterInches;

    // Calculate velocity (in inches per second)
    float velocityInchesPerSec = circumferenceInches / periodSeconds;

    // Convert velocity to miles per hour
    // float velocityKph = (velocityInchesPerSec * 3600) / (12 * 5280) * 1.60934;
    float velocityMph = (velocityInchesPerSec * 3600) / (12 * 5280);

    DB_PRINT("Speed:");
    DB_PRINTLN(velocityMph);
    return velocityMph;
  } else {
    return -1;  // Return -1 if period is not valid
  }
}

void measurePeriodTACH() {  // 2 pulses per revolution
  unsigned long currentTime = millis();
  periodTACH = currentTime - lastTimeTACH;
  lastTimeTACH = currentTime;
}

float calculateRPM() {
  noInterrupts();
  unsigned long periodMillis = periodTACH;
  interrupts();

  if (periodMillis > 0) {
    float frequency = 1.0 / (periodMillis / 1000.0);  // Convert period to seconds and calculate frequency
    float rpm = (frequency * 60.0) / tachPulsesPerRevolution;
    DB_PRINT("RPM: ");
    DB_PRINTLN(rpm);
    return rpm;
  } else {
    return -1;  // Return -1 if period is not valid
  }
}

void readDigital() {
  // read status of digital pins (1-13)
  digitalPins = dig_card.readInputs();

  // int size = sizeof(dig);
  // int bitposition = 0;
  // for (int i = 1; i <= size; i++) {
  //   if (dig_card.readInputs(i) == 1) digitalPins |= (1 << bitposition);
  //   bitposition++;
  // }
  DB_PRINT("Digital: ");
  DB_PRINTLN(digitalPins, BIN);
}

void readGearPosition() {
  gearPosition = adc_card.readAnalogMv(ADC_GEAR);
}


void SendCANFramesToSerial() {
  byte buf[8];

  // build & send CAN frames to RealDash.
  // a CAN frame payload is always 8 bytes containing data in a manner
  // described by the RealDash custom channel description XML file
  // all multibyte values are handled as little endian by default.
  // endianess of the values can be specified in XML file if it is required to use big endian values

  char zeros[6] = { 0 };

  // build 1st CAN frame, VSS, TACH, Gear, TPS (just example data)
  memcpy(buf, &speed, 2);
  memcpy(buf + 2, &rpm, 2);
  memcpy(buf + 4, &gearPosition, 2);
  memcpy(buf + 6, &odometer, 2);

  // write first CAN frame to serial
  SendCANFrameToSerial(3200, buf);

  // build 2nd CAN frame, Temperature
  memcpy(buf, &iaTemp, 2);
  memcpy(buf + 2, &oilTemp, 2);
  memcpy(buf + 4, &coolantTemp, 2);
  memcpy(buf + 6, &ambientTemp, 2);

  // write 2nd CAN frame to serial
  SendCANFrameToSerial(3201, buf);

  // build 3rd CAN frame, Pressure + transTemp
  memcpy(buf, &boostPressure, 2);
  memcpy(buf + 2, &oilPressure, 2);
  memcpy(buf + 4, &fuelPressure, 2);
  memcpy(buf + 6, &EGTemp, 2);

  // write 3rd CAN frame to serialc:\Users\davep\OneDrive\Desktop\RealDash\Arduino\RealDash_CAN\realdash_can_example.xml
  SendCANFrameToSerial(3202, buf);

  // build 4th CAN frame, Acceleration
  memcpy(buf, &accelerationX, 2);
  memcpy(buf + 2, &accelerationY, 2);
  memcpy(buf + 4, &accelerationZ, 2);
  memcpy(buf + 6, &zeros, 2);

  SendCANFrameToSerial(3203, buf);

  // build 4th CAN frame, cruise
  memcpy(buf, &cruiseActive, 2);
  memcpy(buf + 2, &cruiseSetValue, 2);
  memcpy(buf + 4, &digitalPins, 2);
  memcpy(buf + 6, &zeros, 2);

  // write 4th CAN frame to serial
  SendCANFrameToSerial(3204, buf);
}

void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData) {
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
  Serial1.write(serialBlockTag, 4);

  // the CAN frame id number (as 32bit little endian value)
  Serial1.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  Serial1.write(frameData, 8);
}
