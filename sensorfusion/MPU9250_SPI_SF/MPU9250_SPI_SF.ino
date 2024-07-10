#include "mpu9250.h"		// https://github.com/bolderflight/MPU9250 	 		// needed for the Serial output https://github.com/geneReeves/ArduinoStreaming
#include "SensorFusion.h"
SF fusion;


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
LowPass<2> a_x(3,1e3,true);
LowPass<2> a_y(3,1e3,true);
LowPass<2> a_z(3,1e3,true);

float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float pitch, roll, yaw;
float deltat;


bfs::Mpu9250 imu;

int status;

#define EULER_DATA
//#define RAW_DATA
//#define PROCESSING
//#define SERIAL_PLOTER


void setup() {
  // serial to display data
  Serial.begin(115200);
    Wire.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  // start communication with IMU
  status = imu.Begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
    if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
}

double accel_fit(double x, double a, double b) {
  return (a*x)+b;
}


void loop() {

  if (imu.Read()){

  ax = a_y.filt((accel_fit(imu.accel_y_mps2()/9.80665,0.99906226,+0.00205141)));
  ay = a_x.filt((accel_fit(-imu.accel_x_mps2()/9.80665,-0.99951859,-0.0122441)));
  az = a_z.filt((accel_fit(imu.accel_z_mps2()/9.80665,-1.00036038,  2.66762774e-04)));
  gx = imu.gyro_y_radps()+0.04;
  gy = -(imu.gyro_x_radps()-0.04);
  gz = imu.gyro_z_radps();
  mx = imu.mag_y_ut()-18.81;
  my = imu.mag_x_ut()-66.95;
  mz = imu.mag_z_ut()-16.36;
  temp = imu.die_temp_c();

  deltat = fusion.deltatUpdate();
  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony is suggested if there isn't the mag
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, deltat);  //else use the magwick

  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();

  Serial.print(yaw);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(roll);
  }



  //delay(200); //for readability

}
