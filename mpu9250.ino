#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <MPU9250.h>

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu("/imu_data", &imu_msg);

MPU9250 mpu;

void setup() {
  nh.initNode();
  nh.advertise(pub_imu);
  
  // Initialize MPU9250
  mpu.setup(0x68); // Change address if necessary
  delay(5000); // Allow time for initialization
  
  // Calibrate sensors
  mpu.calibrateAccelGyro();
  mpu.calibrateMag();
}

void loop() {
  if (mpu.update()) {
    // Get orientation angles
    float roll = mpu.getRoll();
    float pitch = mpu.getPitch();
    float yaw = mpu.getYaw();
    
    // Convert Euler angles to quaternion
    float qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
    float qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    float qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    float qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    
    // Get angular velocities
    float gyroX = mpu.getGyroX() * (M_PI / 180.0); // Convert to rad/s
    float gyroY = mpu.getGyroY() * (M_PI / 180.0);
    float gyroZ = mpu.getGyroZ() * (M_PI / 180.0);
    
    // Get linear accelerations
    float accelX = mpu.getAccelX() * 9.81; // Convert to m/s^2
    float accelY = mpu.getAccelY() * 9.81;
    float accelZ = mpu.getAccelZ() * 9.81;
    
    // Populate IMU message
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_link";
    
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.orientation.w = qw;
    
    imu_msg.angular_velocity.x = gyroX;
    imu_msg.angular_velocity.y = gyroY;
    imu_msg.angular_velocity.z = gyroZ;
    
    imu_msg.linear_acceleration.x = accelX;
    imu_msg.linear_acceleration.y = accelY;
    imu_msg.linear_acceleration.z = accelZ;
    
    // Covariance matrices are set to unknown (all zeros)
    for (int i = 0; i < 9; i++) {
      imu_msg.orientation_covariance[i] = 0;
      imu_msg.angular_velocity_covariance[i] = 0;
      imu_msg.linear_acceleration_covariance[i] = 0;
    }
    
    pub_imu.publish(&imu_msg);
    
    nh.spinOnce();
    delay(10); // Adjust for desired publish rate
  }
}
