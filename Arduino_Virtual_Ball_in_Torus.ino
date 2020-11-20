// ================================================================
// ===               MPU6050  Variables                         ===
// ================================================================

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_eulerARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion sensorQ;     // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [phi, theta, psi]    Euler angle container


// ================================================================
// ===               GyroBall Variables                         ===
// ================================================================

#define DEBUG true
#define DEBUG_LAMP false
#define DEBUG_PERF false
#define DEBUG_DERIV_ANGLE false
#define DEBUG_PP false

#define CALLIBRATION_STEPS 400 //100 means 1 second

#define SENSORS_GRAVITY_EARTH (9.80665F) /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON (1.6F)      /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN (275.0F)     /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)

int loopWithDataCount = 0;
Quaternion sensorQ0;
Quaternion sensorQ0Conjugate;
float accel_scale;      // based on MPU6050_RANGE_x_G
float alpha = 0;
float alpha_p = 0;
float l = 0.04; // 4cm, radius of NeoPixel
float g = 9.81;
float friction_factor = .002;
float previous_now = 0;
float loop_now = 0;
float dt = 0;
float previous_sin_theta = 0;
float previous_sin_phi = 0;
float previous_sin_psi = 0;
float previous_cos_theta = 0;
float previous_cos_phi = 0;
float previous_cos_psi = 0;
float previous_phi_p = 0;
float previous_psi_p = 0;


// ================================================================
// ===               NeoPixel Variables                         ===
// ================================================================

#include <Adafruit_NeoPixel.h>
#define NEOPIXEL_CONTROL_PIN 6

int neoPixelLampCount = 24;
float lampPerRadian = neoPixelLampCount / (2 * M_PI);
int previousLampNb = 0;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(neoPixelLampCount, NEOPIXEL_CONTROL_PIN, NEO_RBG + NEO_KHZ800);
uint32_t blackColor = strip.Color(0, 0, 0);
uint32_t ballColor = strip.Color(50, 0, 0);


// ================================================================
// ===            NEW EULER METHOD FOR ZXZ SUPPORT              ===
// ================================================================

uint8_t dmpGetEulerZXZ(float *data, Quaternion *q) {
  //From http://bediyap.com/programming/convert-quaternion-to-euler-rotations/ for zxz (Canonical Euler representation)
  data[0] = atan2(2 * (q->x * q->z + q->w * q->y), -2 * (q->y * q->z - q->w * q->x)); // phi ]-pi, pi]
  data[1] = acos(q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z);   // theta [0, pi]
  data[2] = atan2(2 * (q->x * q->z - q->w * q->y), 2 * (q->y * q->z + q->w * q->x)); // psi ]-pi, pi]
  return 0;
}

void debugQuaternion(Quaternion *qu) {
  Serial.print(qu->w);
  Serial.print(F("\t"));
  Serial.print(qu->x);
  Serial.print(F("\t"));
  Serial.print(qu->y);
  Serial.print(F("\t"));
  Serial.print(qu->z);
}


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.setWireTimeout(3000, true); //timeout value in uSec added reading https://github.com/jrowberg/i2cdevlib/issues/543 GUY
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  /*mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);*/
  mpu.setZAccelOffset(1810); // 1688 factory default for my test chip. Mine: 1680

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000); //low precision, high rotation capability
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); //high precision
     mpu.PrintActiveOffsets();
    if (DEBUG) {
      Serial.print(F("getFullScaleGyroRange (3=MPU6050_GYRO_FS_2000, 0=MPU6050_GYRO_FS_250): "));
      Serial.println(mpu.getFullScaleGyroRange());
      Serial.print(F("getFullScaleAccelRange (0=MPU6050_ACCEL_FS_2): "));
      Serial.println(mpu.getFullScaleAccelRange());
    }
    switch (mpu.getFullScaleAccelRange()) {
      case MPU6050_ACCEL_FS_2: //0
        accel_scale = 16384;
        break;
      case MPU6050_ACCEL_FS_4: //1
        accel_scale = 8192;
        break;
      case MPU6050_ACCEL_FS_8: //2
        accel_scale = 4096;
        break;
      case MPU6050_ACCEL_FS_16: //3
        accel_scale = 2048;
        break;
      default:
        accel_scale = 1;
    }
    accel_scale /= (2. * SENSORS_GRAVITY_STANDARD); // 2G

    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  //NeoPixel
  strip.begin(); //TODO: seems no way to check communication is working with NeoPixel...

  //set start time
  previous_now = ((float)millis()) / 1000;
}


// ================================================================
// ===                    LOOP PERFORMANCE                      ===
// ================================================================

void debug_loop_performance() {
  if (loopWithDataCount % 500 == 0) {
    Serial.print(F("loopWithDataCount\t"));
    Serial.print(loopWithDataCount);
    Serial.print(F("\ttime\t"));
    Serial.println(loop_now);
  }
}


// ================================================================
// ===                    CALLIBRATION                          ===
// ================================================================

void callibrateOrientation(Quaternion *q) {
  previous_now = loop_now;

  //light progress of callibration
  strip.setPixelColor(neoPixelLampCount * loopWithDataCount / CALLIBRATION_STEPS, ballColor);
  strip.show();

  //ignore 3/4 of time, wait for sensor to stabilize
  if(loopWithDataCount < (CALLIBRATION_STEPS * 3 / 4))
    return;

  //callibration last step by calculating average and normalization
  if(loopWithDataCount == CALLIBRATION_STEPS) {
    sensorQ0.normalize();
    sensorQ0Conjugate = sensorQ0.getConjugate();
    strip.clear();
    return;
  }

  //accumulate
  sensorQ0.w += q->w;      
  sensorQ0.x += q->x;      
  sensorQ0.y += q->y;      
  sensorQ0.z += q->z;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

    loopWithDataCount++;
    loop_now = ((float)millis()) / 1000;
    dt = loop_now - previous_now;

    if (DEBUG_PERF)
      debug_loop_performance();

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&sensorQ, fifoBuffer);

    if(loopWithDataCount <= CALLIBRATION_STEPS) {
      callibrateOrientation(&sensorQ);
      return;
    }

    Quaternion worldQ = sensorQ.getProduct(sensorQ0Conjugate);
    dmpGetEulerZXZ(euler, &worldQ);
     
    if (DEBUG && loopWithDataCount%100==0) {
      Serial.print(F("t\t"));
      Serial.print((float)(loopWithDataCount / 100));
      Serial.print(F("\tsensorQ\t"));
      debugQuaternion(&sensorQ);
      Serial.print(F("\tworldQ\t"));
      debugQuaternion(&worldQ);
      Serial.print(F("\teuler\t"));
      Serial.print(euler[0] * 180 / M_PI);
      Serial.print(F("\t"));
      Serial.print(euler[1] * 180 / M_PI);
      Serial.print(F("\t"));
      Serial.print(euler[2] * 180 / M_PI);
      Serial.print(F("\t"));
    }

    if (isnan(euler[0]) || isnan(euler[1]) || isnan(euler[2])) {
      if (DEBUG && loopWithDataCount%100==0)
        Serial.println();
      return;
    }

    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &sensorQ);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &sensorQ);
    
    if (DEBUG && loopWithDataCount%100==0) {
      Serial.print(F("aa\t"));
      Serial.print((float)aa.x / accel_scale);
      Serial.print(F("\t"));
      Serial.print((float)aa.y / accel_scale);
      Serial.print(F("\t"));
      Serial.print((float)aa.z / accel_scale);
      Serial.print(F("\ta="));
      Serial.print(sqrt(pow(aa.x,2)+pow(aa.y,2)+pow(aa.z,2)) / accel_scale);
      Serial.print(F("\t"));

      Serial.print(F("aaReal\t"));
      Serial.print(aaReal.x / accel_scale);
      Serial.print(F("\t"));
      Serial.print(aaReal.y / accel_scale);
      Serial.print(F("\t"));
      Serial.print(aaReal.z / accel_scale);
      Serial.print(F("\t"));

      Serial.print(F("aaWorld\t"));
      Serial.print(aaWorld.x / accel_scale);
      Serial.print(F("\t"));
      Serial.print(aaWorld.y / accel_scale);
      Serial.print(F("\t"));
      Serial.print(aaWorld.z / accel_scale);
      Serial.print(F("\t"));
    }

    float phi = euler[0];
    float theta = euler[1];
    float psi = euler[2];
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);
    float sin_phi = sin(phi);
    float cos_phi = cos(phi);
    float cos_psi_plus_alpha = cos(psi + alpha);
    float sin_psi_plus_alpha = sin(psi + alpha);

    //Monent of gravity, in non galiean strip referential
    float alpha_pp_gravity = (-sin_theta * cos_psi_plus_alpha * g) / l;

    //Monent of inertia strength
    float alpha_pp_inertia = (
                  (aaWorld.x / accel_scale) * (sin_phi * cos_theta * cos_psi_plus_alpha + cos_phi * sin_psi_plus_alpha) +
                  (aaWorld.y / accel_scale) * (-cos_phi * cos_theta * cos_psi_plus_alpha + sin_phi * sin_psi_plus_alpha) +
                  (aaWorld.z / accel_scale) * (-sin_theta * cos_psi_plus_alpha)
                ) / l;

    //Monent of centrifuge strength
    float theta_p = 0;
    if (abs(cos_theta) > .5)
      theta_p = (sin_theta - previous_sin_theta) / cos_theta / dt;
    else
      theta_p = -(cos_theta - previous_cos_theta) / sin_theta / dt;
    float phi_p = 0;
    if (abs(cos_phi) > .5)
      phi_p = (sin_phi - previous_sin_phi) / cos_phi / dt;
    else
      phi_p = -(cos_phi - previous_cos_phi) / sin_phi / dt;
    float phi_p_sin_theta = phi_p * sin_theta;
    if (DEBUG_DERIV_ANGLE) {
      Serial.print(F("theta_p\t"));
      Serial.print(theta_p);
      Serial.print(F("\tphi_p\t"));
      Serial.print(phi_p);
      Serial.print(F("\tcos_theta\t"));
      Serial.print(cos_theta);
      Serial.print(F("\tcos_phi\t"));
      Serial.print(cos_phi);
      if (abs(theta_p) > 20 || abs(phi_p) > 20)
        Serial.print(F("***"));
      Serial.println();
    }
    float alpha_pp_centrifuge = -(theta_p * cos_psi_plus_alpha + phi_p_sin_theta * sin_psi_plus_alpha) * (-theta_p * sin_psi_plus_alpha + phi_p_sin_theta * cos_psi_plus_alpha);

    //Monent of Euler strength
    float sin_psi = sin(psi);
    float cos_psi = cos(psi);
    float psi_p = 0;
    if (abs(cos_psi) > .5)
      psi_p = (sin_psi - previous_sin_psi) / cos_psi / dt;
    else
      psi_p = -(cos_psi - previous_cos_psi) / sin_psi / dt;
    float phi_pp = (phi_p - previous_phi_p) / dt;
    float psi_pp = (psi_p - previous_psi_p) / dt;
    float alpha_pp_euler = -(phi_pp * cos_theta + psi_pp - theta_p * phi_p_sin_theta);

    //sum all accelerations
    if (DEBUG_PP) {
      Serial.print(F("alpha_pp_gravity\t"));
      Serial.print(alpha_pp_gravity);
      Serial.print(F("\talpha_pp_inertia\t"));
      Serial.print(alpha_pp_inertia);
      Serial.print(F("\talpha_pp_centrifuge\t"));
      Serial.print(alpha_pp_centrifuge);
      Serial.print(F("\talpha_pp_euler\t"));
      Serial.println(alpha_pp_euler);
    }
    
    float alpha_pp = alpha_pp_gravity + alpha_pp_inertia + alpha_pp_centrifuge + alpha_pp_euler;
    //float alpha_pp = alpha_pp_gravity;

    //Integrate alpha and apply friction
    alpha_p += dt * alpha_pp - friction_factor * alpha_p;

    //Integrate alpha
    alpha += dt * alpha_p;

    //Adjust for range -pi < alpha <= pi, aligned with theta. It is optional
    if(alpha > M_PI)
      alpha -= 2. * M_PI;
    else if(alpha <= -M_PI)
      alpha += 2. * M_PI;

    //Light the lamp
    //Ring (is lamps up) is clock-wise which z axis viewed from top is counter clock-wise
    int lampNb = ((int)(neoPixelLampCount - alpha * lampPerRadian)) % neoPixelLampCount;
    if (DEBUG_LAMP) {
      Serial.print(F("lamp "));
      Serial.print(lampNb);
      Serial.print(F("\talpha\t"));
      Serial.print(alpha);
      Serial.print(F("\talpha_p\t"));
      Serial.println(alpha_p);
    }
    strip.setPixelColor(previousLampNb, blackColor);
    strip.setPixelColor(lampNb, ballColor);
    strip.show();
    previousLampNb = lampNb;

    if (DEBUG && loopWithDataCount%100==0)
      Serial.println();

    // blink Arduino LED to indicate activity, every 1 second (interupt is 100Hz)
    if(loopWithDataCount % 100 == 0) {
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }

    //set 'previous' variables
    previous_now = loop_now;
    previous_sin_theta = sin_theta;
    previous_sin_phi = sin_phi;
    previous_sin_psi = sin_psi;
    previous_cos_theta = cos_theta;
    previous_cos_phi = cos_phi;
    previous_cos_psi = cos_psi;
    previous_phi_p = phi_p;
    previous_psi_p = psi_p;
  }
}
