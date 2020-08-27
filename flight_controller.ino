#include <Servo.h>
#include <Wire.h>
#define GYRO_PART 0.995
#define ACC_PART 0.005
using namespace std;
//change unessesary floats to ints
//Servo m1;
class Position {
  public:
    float gyroPart = 0.02 , accelPart = 0.98;
    float RAD_TO_DEGREES = 180 / M_PI;
    float gForceX, gForceY, gForceZ;
    long accelX, accelY, accelZ;
    long gyroX, gyroY, gyroZ;
    long fX = 0, fY = 0;
    float rotX, rotY, rotZ;
    float xAngle = 0, yAngle = 0, zAngle = 0;
    float axAngle, ayAngle, azAngle;
    float gyroXC = 0, gyroYC = 0, gyroZC = 0;
    float accelXC = 0, accelYC = 0, accelZC = 0;
    float lastUpdate, printTime = 0; //last update time in microseconds since program started, consider making different time for each angle being updated

    void calibrateGyro() {
      float xTot = 0, yTot = 0, zTot = 0;
      for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 50; j++) {
          getGyroValues(false);
          xTot += rotX;
          yTot += rotY;
          zTot += rotZ;
        }
        delay(20);
      }
      gyroXC = xTot / 1000;
      gyroYC = yTot / 1000;
      gyroZC = zTot / 1000;
    }


    void setupMPU() {
      Wire.beginTransmission(0b1101000); //i2c adress of the mpu
      Wire.write(0x6B);
      Wire.write(0b00000000);
      Wire.endTransmission();
      Wire.beginTransmission(0b1101000);
      Wire.write(0x1B);
      Wire.write(0b00000000);
      Wire.endTransmission();
      Wire.beginTransmission(0b1101000);
      Wire.write(0x6B);
      Wire.write(0b00000000);
      Wire.endTransmission();
      calibrateGyro();
      calibrateAccel();
    }


    void getGyroValues(bool adjusted = true) {
      Wire.beginTransmission(0b1101000); //i2c adress of the mpu
      Wire.write(0x43);
      Wire.endTransmission();
      Wire.requestFrom(0b1101000, 6); //request gyro registers
      while (Wire.available() <  6);
      gyroX = Wire.read() << 8 | Wire.read();
      gyroY = Wire.read() << 8 | Wire.read();
      gyroZ = Wire.read() << 8 | Wire.read();
      processGyroData();
      if (adjusted) {
        adjustGyro();
      }
    }

    void printGyro() {
      Serial.print("Gyroscope- X: ");
      Serial.print(rotX );
      Serial.print("  Y: ");
      Serial.print(rotY );
      Serial.print("  Z: ");
      Serial.println(rotZ );
    }

    void processGyroData() {
      rotX = gyroX / 131.0 - 40;
      rotY = gyroY / 131.0;
      rotZ = gyroZ / 131.0;
    }


    void calibrateAccel() {
      float xTot = 0, yTot = 0, zTot = 0;
      for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 10; j++) {
          getAccelValues(false);
          xTot += gForceX;
          yTot += gForceY;
          zTot += gForceZ;
        }
        delay(1);
      }
      accelXC = xTot / 50;
      accelYC = yTot / 50;
      accelZC = 1 - (zTot / 50);
    }




    void getAccelValues(bool adjusted = true) {
      Wire.beginTransmission(0b1101000);
      Wire.write(0x3B); // register for accel readings
      Wire.endTransmission();
      Wire.requestFrom(0b1101000, 6); //request accel registers
      while (Wire.available() <  6);
      accelX = Wire.read() << 8 | Wire.read();
      accelY = Wire.read() << 8 | Wire.read();
      accelZ = Wire.read() << 8 | Wire.read();
      processAccelData();
      if (adjusted) {
        adjustAccel();
      }
    }


    void printAccel() { // will be innacurate if used after asjust gyro
      Serial.print("Accelerometer- X: ");
      Serial.print(gForceX);
      Serial.print("  Y: ");
      Serial.print(gForceY);
      Serial.print("  Z: ");
      Serial.println(gForceZ);
    }

    void processAccelData() {
      gForceX = accelX / 16384.0;
      gForceY = accelY / 16384.0;
      gForceZ = accelZ / 16384.0;
    }

    void adjustGyro() {
      rotX = rotX - gyroXC;
      rotY = rotY - gyroYC;
      rotZ = rotZ - gyroZC;
    }

    void adjustAccel() {
      gForceX = gForceX - accelXC;
      gForceY = gForceY - accelYC;
      gForceZ = gForceZ + accelZC;
    }

    void getfilteredAccel() {
      float xVals [11] = {0 , 0, 0, 0, 0 , 0, 0, 0, 0 , 0, 0};
      float yVals [11] = {0 , 0, 0, 0, 0 , 0, 0, 0, 0 , 0, 0};
      float zVals [11] = {0 , 0, 0, 0, 0 , 0, 0, 0, 0 , 0, 0};
      for (int i = 0; i < 11; i++) {
        getAccelValues();
        xVals[i] = gForceX;
        yVals[i] = gForceY;
        zVals[i] = gForceZ;
      }

      gForceX = getMean(xVals);
      gForceY = getMean(yVals);
      gForceZ = getMean(zVals);
    }


    void adjust_angle() {
      float timeMicroS = micros() - lastUpdate;
      lastUpdate = micros();
      getGyroValues();
      getfilteredAccel();
      axAngle = atan2(gForceY, gForceZ) * RAD_TO_DEGREES;
      ayAngle = atan2(-1 * gForceX, sqrt(gForceY * gForceY + gForceZ * gForceZ)) * RAD_TO_DEGREES;


      fX = gyroPart * (fX + (rotX * (timeMicroS / 1000000))) + accelPart * axAngle;
      fY = gyroPart * (fY + (rotY * (timeMicroS / 1000000))) + accelPart * ayAngle;
      
      
      if (millis() - printTime >= 1000) {
        printTime = millis();
        
        //printGyro();
        //printAccel();
      }

    }


    static float getMean(float numbers[], int count = 11)
    {
      float tot = 0;
      for (int i = 0; i < count; i++) {
        tot += numbers[i];
      }
      return (tot / count);
    }

    static float getMedian(float numbers[], int count = 11) 
    {
      int i = 1;
     while ( i < count ) {
           for ( int j = i; numbers[j - 1] > numbers[j]; j-- ) {
                int temp = numbers[j - 1];
                numbers[j - 1] = numbers[j];
                numbers[j] = temp;
              } 
              i++;
           }
    /* for ( int q = 0; q < 11; q++ ){
    Serial.print(numbers[q]);
    Serial.print(", ");
    }
    Serial.println(""); */
    return numbers[5];
    }
};

class Receiver {
  public:
    long gyroX, gyroY, gyroZ;
    int com1 = 12, com2 = 11, com3 = 3, com4 = 4; // reciever pins
    float adjust1 = 0, adjust2 = 0, adjust3 = 0;
    float Ch1 = 0, Ch2 = 0, Ch3 = 0, Ch4 = 0;
    float throttle, roll, pitch, yaw;

    void readChannels(float maxSpeed = 2000) {
      int sums [4] = {0 , 0, 0, 0};
      int samples [4] = {0, 0, 0, 0};
      Ch1 = 0;
      Ch2 = 0;
      Ch3 = 0;
      Ch4 = 0;
      while (Ch1 == 0 || Ch2 == 0 || Ch3 == 0 || Ch4 == 0) {
        if (Ch1 == 0) {
          Ch1 = pulseIn(com1, HIGH, 25000);
        }
        if (Ch2 == 0) {
          Ch2 = pulseIn(com2, HIGH, 25000);
        }
        if (Ch3 == 0) {
          Ch3 = pulseIn(com4, HIGH, 25000);
        }
        if (Ch4 == 0) {
          Ch4 = pulseIn(com3, HIGH, 25000);
        }
      }
      Ch1 += adjust1;
      Ch2 += adjust2;
      Ch3 += adjust3;
      throttle = map(Ch4, 1000, 2000, 1000, maxSpeed);
      roll = map(Ch1, 1000, 2000, -25, 25);
      pitch = map(Ch2, 1000, 2000, -25, 25);
      yaw = map(Ch3, 1000, 2000, -50, 50);
    }




    void setupCom() {
      pinMode(com1, INPUT);
      pinMode(com2, INPUT);
      pinMode(com3, INPUT);
      pinMode(com4, INPUT);
      Serial.print(1400);
      float sum1 = 0, sum2 = 0, sum3 = 0;
      for (int i = 0; i < 20; i++) {
        readChannels();
        sum1 += Ch1;
        sum2 += Ch2;
        sum3 += Ch3;
      }
      adjust1 = 1500 - sum1 / 20;
      adjust2 = 1500 - sum2 / 20;
      adjust3 = 1500 - sum3 / 20;
      Serial.print(1400);
    }

    void printRC() {
      Serial.print("RC- Ch1: ");
      Serial.print(Ch1);
      Serial.print(" Ch2 ");
      Serial.print(Ch2);
      Serial.print(" Ch3 ");
      Serial.print(Ch3);
      Serial.print(" CH4 ");
      Serial.println(Ch4);
    }

    void printAngles() {
      Serial.print("RC- Throttle: ");
      Serial.print(throttle);
      Serial.print(" Roll: ");
      Serial.print(roll);
      Serial.print(" Pitch ");
      Serial.print(pitch);
      Serial.print(" Yaw ");
      Serial.println(yaw);

    }
};

class Motors {
  public:
    Servo m1, m2, m3, m4; //motors

    void setupEsc() {
      m1.writeMicroseconds(0);
      m2.writeMicroseconds(0);
      m3.writeMicroseconds(0);
      m4.writeMicroseconds(0);
    }

    void set(int motNum, float Speed) {
      if (motNum == 1) {
        m1.writeMicroseconds(Speed);
      }
      if (motNum == 2) {
        m2.writeMicroseconds(Speed);
      }
      if (motNum == 3) {
        m3.writeMicroseconds(Speed);
      }
      if (motNum == 4) {
        m4.writeMicroseconds(Speed);
      }
    }

    void startMotors() {
      bool started = false;
      int Speed = 900;
      float timeMicro = micros();
      while (!started) {
        // put your main code here, to run repeatedly:
        m1.writeMicroseconds(Speed);
        m2.writeMicroseconds(Speed);
        m3.writeMicroseconds(Speed);
        m4.writeMicroseconds(Speed);
        if (micros() -  timeMicro > 100) {
          Speed += 1;
          timeMicro = micros();
        }
        if (Speed > 1000) {
          started = true;
        }
      }
    }

};


class Controller {
  public:
    Position pos;
    Receiver receiver; 
    Motors motors;
    bool on = false;
    int led = 8;
    float Kp[3]        = {1.3, 1.5, 1.5};    // Yaw, Pitch, Roll (4, 1.3, 1.3)
    float Ki[3]        = {0, 0, 0}; // Yaw, Pitch, Roll (.02, .04, .04)
    float Kd[3]        = {0, 0, 0};        // Yaw, Pitch, Roll (0,18, 18)
    float delta_err[3] = {0, 0, 0};          // Yaw, Pitch, Roll
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;
    int YAW = 0, PITCH = 1, ROLL = 2, THROTTLE = 3;// for the sake of selecting the right indexes of lists
    float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
    float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
    float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
    float printTime = 0;
    /*
       replace with my variables// probably

       float instruction[4];
      float measures[3] = {0, 0, 0};*/
    unsigned long pulse_length_esc[4] = {1000, 1000, 1000, 1000};



    void cycle(float maxSpeed = 2000) {
      receiver.readChannels(maxSpeed);
      check_on();
      if (on) {   
        pos.adjust_angle();
        PID(maxSpeed);
        setMotors();
      }
    }

    void check_on() {
      if (receiver.Ch4 < 1040 and receiver.Ch3 < 1040) {
        on = false;
        digitalWrite(led, LOW);
        motors.set(1, 0);
        motors.set(2, 0);
        motors.set(3, 0);
        motors.set(4, 0);
      }
      else if (receiver.Ch4 < 1040 and receiver.Ch3 > 1960) {
        on = true;
        digitalWrite(led, HIGH);
      }
    }

    void PID(unsigned long maxSpeed, bool level=true) { //borrowed from this code on github https://github.com/lobodol/drone-flight-controller/blob/master/drone-flight-controller.ino
      if (level){
      errors[YAW]   = receiver.yaw   - pos.rotZ;
      errors[PITCH] = receiver.pitch - pos.fX;
      errors[ROLL]  = receiver.roll  - pos.fY; 
      }
      else {
      errors[YAW]   = 0   - pos.rotZ;
      errors[PITCH] = 0 - pos.fX;
      errors[ROLL]  = 0  - pos.fY;
      }
      
      

    
      // Do not calculate anything if throttle is 0
      if (receiver.throttle >= 1012) {
        // Calculate sum of errors : Integral coefficients
        error_sum[YAW]   += errors[YAW];
        error_sum[PITCH] += errors[PITCH];
        error_sum[ROLL]  += errors[ROLL];

        // Calculate error delta : Derivative coefficients
        delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
        delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
        delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

        // Save current error as previous_error for next time
        previous_error[YAW]   = errors[YAW];
        previous_error[PITCH] = errors[PITCH];
        previous_error[ROLL]  = errors[ROLL];

        // PID = e.Kp + ∫e.Ki + Δe.Kd
        yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
        roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

        // Calculate pulse duration for each ESC
        pulse_length_esc[0] = receiver.throttle + roll_pid - pitch_pid + yaw_pid;//red prop black top
        pulse_length_esc[1] = receiver.throttle - roll_pid - pitch_pid - yaw_pid -40;//red prop grey top
        pulse_length_esc[2] = receiver.throttle - roll_pid + pitch_pid + yaw_pid;// white prop black top
        pulse_length_esc[3] = receiver.throttle + roll_pid + pitch_pid - yaw_pid;//white prop grey top


        if (millis() - printTime > 100){
        printAngle();
        printPID();
        printTime = millis();
        }

        if (pos.fX > 90 || pos.fX < -90 || pos.fY > 90 || pos.fY < -90){
        pulse_length_esc[0] = 0; 
        pulse_length_esc[1] = 0; 
        pulse_length_esc[2] = 0; 
        pulse_length_esc[3] = 0; 
        on = false;
        }

        for (int i = 0; i < 4; i++) {
          if(pulse_length_esc[i] > maxSpeed)
          {
            pulse_length_esc[i] = maxSpeed;
            }
        }
       
      }
    }

    void printAngle(){
      Serial.print("X: ");
      Serial.print(pos.fX);
      Serial.print("  Y: ");
      Serial.println(pos.fY);
      Serial.print("  Z: ");
      Serial.println(pos.rotZ);
      }

    void printPID(){
      Serial.print("0: ");
      Serial.print(pulse_length_esc[0]);
      Serial.print("  1: ");
      Serial.print(pulse_length_esc[1]);
      Serial.print(" 2: ");
      Serial.print(pulse_length_esc[2]);
      Serial.print("  3: ");
      Serial.println(pulse_length_esc[3]);
      }
    void setMotors() {
      motors.set(1, pulse_length_esc[0]);
      motors.set(2, pulse_length_esc[1]);
      motors.set(3, pulse_length_esc[2]);
      motors.set(4, pulse_length_esc[3]);
    }
};


Controller c;

void setup() {
  Serial.begin(9600);
  pinMode(c.led, OUTPUT);
  c.receiver.setupCom();
  while (!c.on) {
    c.receiver.readChannels(2000);
    c.check_on();
    Serial.print("Ch4: ");
    Serial.println(c.receiver.Ch4);
    Serial.print("Ch3: ");
    Serial.println(c.receiver.Ch3);
  }
  Serial.print(10);

  //m1.attach(6); //Adds ESC to certain pin. arm();
  //m1.writeMicroseconds(0);
  //delay(100);
  c.motors.m1.attach(6); //Adds ESC to certain pin. arm();
  c.motors.m2.attach(9);
  c.motors.m3.attach(10);
  c.motors.m4.attach(5);
  c.pos.setupMPU();
  c.motors.setupEsc();
  c.motors.startMotors();
  //think about starting motors before calibrating gyro in order to get more accurate values

}


void loop() {
  //pos.getGyroValues();
  //getMpuData();
  //pos.printGyro();
  //receiver.readChannels();
  //receiver.printAngles();
  //c.pos.adjust_angle();
  //m1.writeMicroseconds(1200);
  //controlThrottle();
  //pos.getfilteredAccel();
  c.cycle(2000);
}


void controlThrottle() {
  c.receiver.readChannels();
  c.motors.set(1, c.receiver.Ch4 - 800);
  c.motors.set(2, c.receiver.Ch4 - 800);
  c.motors.set(3, c.receiver.Ch4 - 800);
  c.motors.set(4, c.receiver.Ch4 - 800);
}


