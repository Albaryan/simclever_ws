#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"

template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

// Filter instance
LowPass<2> lp_x(3,1e3,true);
LowPass<2> lp_y(3,1e3,true);

MPU6050 gyro;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t gyroIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
String dataToSend;

volatile bool gyroInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  gyroInterrupt = true;
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); //I2C haberleşmesini başlat
  Wire.setClock(400000);

  Serial.begin(115200); //UART haberleşmesini başlat

  gyro.initialize(); //Gyro'yu başlat
  pinMode(INTERRUPT_PIN, INPUT);

  while (!gyro.testConnection());

  delay(100);

  devStatus = gyro.dmpInitialize();

  //Gyro offset değerlerini ayarla
  gyro.setXGyroOffset(35);
  gyro.setYGyroOffset(-3);
  gyro.setZGyroOffset(-50);
  gyro.setXAccelOffset(-3049);
  gyro.setYAccelOffset(839);
  gyro.setZAccelOffset(769);

  if(devStatus == 0) {

  gyro.CalibrateAccel(6);
  gyro.CalibrateGyro(6);

  gyro.setDMPEnabled(true);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  gyroIntStatus = gyro.getIntStatus();

  dmpReady = true;

  packetSize = gyro.dmpGetFIFOPacketSize();
  //

  } else {
            // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
  }

  Serial.print("#");
}

void loop() {
  // put your main code here, to run repeatedly:

  delay(500);

  while(1){

  if (!dmpReady) return;

  if (gyro.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    gyro.dmpGetQuaternion(&q, fifoBuffer);
    gyro.dmpGetGravity(&gravity, &q);
    gyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
    gyro.dmpGetAccel(&aa, fifoBuffer);
    gyro.dmpGetGravity(&gravity, &q);
    gyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    gyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    

    dataToSend="";

    dataToSend.concat(String(ypr[0]));
    dataToSend.concat("|");
    dataToSend.concat(String(round(lp_x.filt(aaWorld.x)/1500)));
    dataToSend.concat("|");
    dataToSend.concat(String(round(lp_y.filt(aaWorld.y)/800)));
    dataToSend.concat("#");

    Serial.print(dataToSend);
  }
  }
}
