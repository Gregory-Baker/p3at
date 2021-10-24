#include <Adafruit_BNO08x.h>
//#include <ros.h>
#include <sensor_msgs/Imu.h>

#define BNO08X_RESET -1

const int reportIntervalUs = 5000;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

sh2_SensorId_t reportTypeRV = SH2_GAME_ROTATION_VECTOR;
sh2_SensorId_t reportTypeAcc = SH2_ACCELEROMETER;
sh2_SensorId_t reportTypeGyro = SH2_GYROSCOPE_CALIBRATED;

class Imu {
  
  private:
    bool initImu();
    void setCovariances();
    void setReports();
    void initMsg();
    sensor_msgs::Imu imu_msg_;
    const bool rv_;
    const bool acc_;
    const bool gyro_;
    
  public:
    Imu(const bool, const bool, const bool);
    void pubImu();
    bool imuOn;
    ros::Publisher pub_;
    void updateImuMsg();
    
};

Imu::Imu(const bool rv, const bool acc, const bool gyro)
  : pub_("imu", &imu_msg_)
  , rv_(rv)
  , acc_(acc)
  , gyro_(gyro)
{
  imuOn = initImu();
  if (imuOn) {
    setReports();
  }
  setCovariances();
}

void Imu::setCovariances() {
  imu_msg_.header.frame_id = "imu";
  if (rv_) {
    imu_msg_.orientation_covariance[0] = 0.001;
    imu_msg_.orientation_covariance[4] = 0.001;
    imu_msg_.orientation_covariance[8] = 0.001;
  }
  else {
    imu_msg_.orientation_covariance[0] = -1;
  }
  
  if (gyro_) {
    imu_msg_.angular_velocity_covariance[0] = 0.02;
    imu_msg_.angular_velocity_covariance[4] = 0.02;
    imu_msg_.angular_velocity_covariance[8] = 0.02;
  }
  else {
    imu_msg_.angular_velocity_covariance[0] = -1;
  }
  
  if (acc_) {
    imu_msg_.linear_acceleration_covariance[0] = 0.04;
    imu_msg_.linear_acceleration_covariance[4] = 0.04;
    imu_msg_.linear_acceleration_covariance[8] = 0.04;
  }
  else {
    imu_msg_.linear_acceleration_covariance[0] = -1;
  }
}

bool Imu::initImu() {
  if (!bno08x.begin_I2C()) {
    return false;
  }
  return true;
}


void Imu::setReports() {
  if (rv_) {
    bno08x.enableReport(reportTypeRV, reportIntervalUs);
  }
  if(gyro_) {
    bno08x.enableReport(reportTypeGyro, reportIntervalUs);
  }
  if(acc_) {
    bno08x.enableReport(reportTypeAcc, reportIntervalUs);
  }
}

void Imu::updateImuMsg()
{
  if (bno08x.wasReset()) {
    setReports();
  }
  
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  
  switch (sensorValue.sensorId) {

    case SH2_ACCELEROMETER:
      imu_msg_.linear_acceleration.x = sensorValue.un.accelerometer.x;
      imu_msg_.linear_acceleration.y = sensorValue.un.accelerometer.y;
      imu_msg_.linear_acceleration.z = sensorValue.un.accelerometer.z;
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      imu_msg_.angular_velocity.x = sensorValue.un.gyroscope.x;
      imu_msg_.angular_velocity.y = sensorValue.un.gyroscope.y;
      imu_msg_.angular_velocity.z = sensorValue.un.gyroscope.z;
      break;
    case SH2_GAME_ROTATION_VECTOR:
      imu_msg_.orientation.w = sensorValue.un.gameRotationVector.real;
      imu_msg_.orientation.x = sensorValue.un.gameRotationVector.i;
      imu_msg_.orientation.y = sensorValue.un.gameRotationVector.j;
      imu_msg_.orientation.z = sensorValue.un.gameRotationVector.k;
      break;
  }
}

void Imu::pubImu() {
  if (imuOn) {
    imu_msg_.header.stamp = nh.now();
    pub_.publish(&imu_msg_);
  }
}
