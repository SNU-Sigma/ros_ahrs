#include "Arduino.h"
#include "ros.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MadgwickAHRS.h"
#include "geometry_msgs/Quaternion.h"

ros::NodeHandle nh;
geometry_msgs::Quaternion imu_msg;
ros::Publisher p("/imu/Quaternion",&imu_msg);

Madgwick filter;
MPU6050 mpu;
HMC5883L hmc;
unsigned long microsPerReading, microsPrevious, loop_cnt;
float accelScale, gyroScale;

int aix, aiy, aiz;
int gix, giy, giz;
int16_t mix, miy, miz;
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
unsigned long microsNow;

int gix_offset = 0;
int giy_offset = 0;
int giz_offset = 0;

float convertRawAcceleration(int aRaw);
float convertRawGyro(int gRaw);

void setup() {
  nh.initNode();
  Serial.begin(1000000);

  Wire.begin();
  nh.advertise(p);

  nh.spinOnce();
  // start the IMU and filter
  filter.begin(200);
  mpu.initialize();
  hmc.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
  hmc.setSampleAveraging(HMC5883L_AVERAGING_1);
  hmc.setDataRate(HMC5883L_RATE_75);

  // // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 200;
  microsPrevious = micros();


  for(int i=0; i<10; i++){
    mpu.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);
    gix_offset += gix*0.1;
    giy_offset += giy*0.1;
    giz_offset += giz*0.1;
  }

}

void loop() {
  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    mpu.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix-gix_offset);
    gy = convertRawGyro(giy-giy_offset);
    gz = convertRawGyro(giz-giz_offset);

    if(loop_cnt%4 == 0){
      hmc.getHeading(&mix, &miy, &miz);
      mx = (float)miy;
      my = -(float)mix;
      mz = (float)miz;
    }

    // update the filter, which computes orientation
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    filter.getQuaternions(imu_msg.w, imu_msg.x, imu_msg.y, imu_msg.z);

    // print the heading, pitch and roll
    p.publish(&imu_msg);
    nh.spinOnce();

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
  loop_cnt++;
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 1000.0) / 32768.0;
  return g;
}
