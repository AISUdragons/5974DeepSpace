package org.usfirst.frc5974.DeepSpace.subsystems;

import org.usfirst.frc5974.DeepSpace.Robot;
import org.usfirst.frc5974.DeepSpace.ADIS16448_IMU;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;

public class Sensors extends Robot{
    //ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    BuiltInAccelerometer accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
    Timer timer = new Timer();
	double xVal;
	double yVal;
	double zVal;
	double angle;
	double rate;
    boolean gyroConnected;
    
    //This is a code example from https://wiki.analog.com/first/adis16448_imu_frc/java.
	public static final double kAngleSetPoint = 0.0; //straight ahead
	public static final double kP = 0.005; //proportional turning constant. not sure what this is, ngl

	//gyro calibration constant, may need to be adjusted. 360 is set to correspond to one full revolution.
	//private static final double kVoltsPerDegreePerSecond=0.0128;

	public static final ADIS16448_IMU FancyIMU = new ADIS16448_IMU();
	double accelX;
	double accelY;
	double accelZ;
	double fancyAngle;
	double angleX;
	double angleY;
	double angleZ;
	double pitch;
	double yaw;
	double roll;
	double fancyRate;
	double rateX;
	double rateY;
	double rateZ;

	double velX;
	double velY;
	double velZ;
	double time;
	double prevTime = 0;
	double dt;

	double gravAngle;

	public void gyroReset() { //it resets the gyro
		FancyIMU.reset();
		//gyro.reset();
	} 

	public void sensorInit() {
		//gyro.calibrate();
		FancyIMU.calibrate();
		velX = velY = velZ = 0;
	}
	public void updateSensors() {
		//ADXRS sensor data
		/*xVal = accel.getX();
		yVal = accel.getY();
		zVal = accel.getZ();
		angle = gyro.getAngle();
		rate = gyro.getRate();
		gyroConnected = gyro.isConnected();*/

		//ADIS sensor data
		accelX = FancyIMU.getAccelX();
		accelY = FancyIMU.getAccelY();
		accelZ = FancyIMU.getAccelZ();
		fancyAngle=FancyIMU.getAngle();
		angleX = FancyIMU.getAngleX();
		angleY = FancyIMU.getAngleY();
		angleZ = FancyIMU.getAngleZ();
		pitch=FancyIMU.getPitch();
		yaw = FancyIMU.getYaw();
		roll =FancyIMU.getRoll();
		fancyRate=FancyIMU.getRate();
		rateX = FancyIMU.getRateX();
		rateY = FancyIMU.getRateY();
		rateZ = FancyIMU.getRateZ();

		time = timer.get();
		dt = time - prevTime;
		velX += accelX * dt;
		velY += accelY * dt;
		velZ += accelZ * dt;
		prevTime = time;

		gravAngle = Math.acos(accelX) * 180/Math.PI;
	}
}