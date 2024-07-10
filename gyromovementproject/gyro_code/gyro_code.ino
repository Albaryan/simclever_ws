#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"
#include <Kalman.h>

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
LowPass<2> lp_z(3,1e3,true);

#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;
int cTime=0,pTime;
double timePast;
double dv=0;
double dx=0;
double errorP;
double errorC;

bool dmpReady = false;
uint8_t gyroIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

volatile bool gyroInterrupt = false;
void dmpDataReady() {
  gyroInterrupt = true;
}

MPU6050 gyro; //Gyro'yu tanımla

//Gyro değişkenlerini tanımla
int16_t ax,ay,az;
//

long double ax_t=0,ay_t=0,az_t=0;

String dataToSend; //Gönderilecek olan veri için değişken oluştur

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); //I2C haberleşmesini başlat
  Wire.setClock(400000);

  Serial.begin(115200); //UART haberleşmesini başlat

  gyro.initialize(); //Gyro'yu başlat
  pinMode(INTERRUPT_PIN, INPUT);
  
  gyro.setFullScaleGyroRange(MPU6050_ACCEL_FS_16);

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

  cTime=millis();

  for (int i=0;i<100;i++){
  
  if (!dmpReady) return;

  if (gyro.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    gyro.dmpGetQuaternion(&q, fifoBuffer);
    gyro.dmpGetAccel(&aa, fifoBuffer);
    gyro.dmpGetGravity(&gravity, &q);
    gyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    gyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    

    double ax_f = round(lp_x.filt(aaWorld.x));
    double ay_f = lp_y.filt(aaWorld.y);
    double az_f = lp_z.filt(aaWorld.z);


    
  
    //Serial.print(dataToSend); //Gönderilecek veri değişkenini kullanarak UART portuna veri gönder

    
    dataToSend=""; //Her döngüde gönderilecek veri değişkenini sıfırla

    
    //dataToSend.concat(String(ax_f));
    //dataToSend.concat("|");
    //dataToSend.concat(String(ay_t));
   // dataToSend.concat("|");
    //dataToSend.concat(String(az_t));
    dataToSend.concat("#");

    ax_t =ax_t+ax_f;
    
    //Serial.print(dataToSend);

    
  }
  
  }
  Serial.println(ax_t/100);
  ax_t=0;
}
