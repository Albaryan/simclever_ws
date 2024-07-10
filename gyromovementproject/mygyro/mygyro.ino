#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

template <int order>
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1];
    float y[order+1]; 

  public:  
    LowPass(float f0, float fs, bool adaptive){

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

      if(adapt){
        setCoef();   
      }
      y[0] = 0;
      x[0] = xn;

      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];
      
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
      
      return y[0];
    }
};
LowPass<2> lp_x(3,1e3,true);
LowPass<2> lp_y(3,1e3,true);


Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial);
    
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);


  Serial.println("#");
  delay(1000);
}

double dt,ct,pt;

double oldX=0,newX;

double oldXv0=0,newXv0;

double oldX0=0, newX0;

double oldY=0,newY;

double oldYv0=0,newYv0;

double oldY0=0, newY0;

double thX=0.25,diffX;

double thXv0=.05,diffXv0;

double thY=0.25,diffY;

double thYv0=.05,diffYv0;

double countEqX=0,countEqY=0;

void loop() {

  ct=millis();
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  dt=(ct-pt)/1000;
  pt=ct;

  // Sinyallerden offset değerlerini çıkar ve filtreden geçir
  newX=lp_x.filt(a.acceleration.x-1.1);
  newY=lp_y.filt(a.acceleration.y+0.075);
  //

  // Algoritma 1
  diffX=newX-oldX; //Elde tutulan bir önceki sinyal ile filtreden geçirilmiş yeni sinyal arasındaki mesafeyi hesapla
  if (abs(newX)<thX) { //Filtreden geçirilmiş yeni sinyalin mutlak değeri threshold'dan küçükse sıfıra eşitle. Böylelikle 0.0'a tam yaklaşmayan sinyal içerisindeki hatalar silinir.
    oldX=0.0;          
  } else if (abs(newX)>3) { //Filtreden geçirilen yeni sinyalin mutlak değeri 3'ten büyükse 3.0'a eşitle. Böylelikle aşırıya giden sinyal değerleri sınırlandırılır.
    if (newX>0) {
      oldX=3.0;
    } else {
      oldX=-3.0;
    }
  } else {  //Sinyalin mutlak değeri 0-3 arasındaysa sinyali eşik değere bağlı olarak aralıklandır. Böylece 0-3 arasındaki sinyalin hata değeri silinir.
    oldX=oldX+(int(diffX/thX)*thX); 
  }

  
  newXv0=oldX*dt + oldXv0; //Elde tutulan ivme değeri ile v= a*t + v0 hız denklemini uygulayarak yeni hız değerini elde et. 
  diffXv0=newXv0-oldXv0; //Hesaplanan yeni hız değeri ile elde tutulan eski hız değeri arasındaki hata farkını al.

  //Algoritma 2
  if (abs(diffXv0)==0.0){ //Üst üste gelen aynı değerleri say
    countEqX++;
  } else {
    countEqX=0;
  }
  if (countEqX==5){ //Eğer üst üste gelen aynı değer sayısı 5'e ulaşmışsa sayacı ve eski hız değerini sıfıra eşitle, 5'e ulaşmamışsa elde tutulan değer yerine yeni hesaplanan değeri yaz.
    countEqX=0;
    oldXv0=0.0;
  } else {
    oldXv0=newXv0;
  }

  newX0=oldXv0*dt*1000; //Hızdan anlık konum hesapla
  oldX0=newX0;

  if (abs(oldX0)>300) { //Hata dolayısıyla anlık konum çok artarsa 0'a eşitle
    oldX0=0.0;
  }

  // X eksenindeki ivme sinyaline uygulanan aynı işlemler
  diffY=newY-oldY;

  if (abs(newY)<thY) {    
    oldY=0.0;
  } else if (abs(newY)>3) {
    if (newY>0) {
      oldY=3.0;
    } else {
      oldY=-3.0;
    }
  } else {
    oldY=oldY+(int(diffY/thY)*thY);
  }
  newYv0=oldY*dt + oldYv0;
  diffYv0=newYv0-oldYv0;
  
  if (abs(diffYv0)==0.0){
    countEqY++;
  } else {
    countEqY=0;
  }
  
  if (countEqY==5){
    countEqY=0;
    oldYv0=0.0;
  } else {
  
  oldYv0=newYv0;
  }
  
  newY0=oldYv0*dt*1000;
  oldY0=newY0;

  if (abs(oldY0)>300) {
    oldY0=0.0;
  }

  
  // Haberleşme bölümü: 
  Serial.print(oldX0);
  Serial.print("|");
  Serial.print(oldY0);  
  Serial.print("#");
}
