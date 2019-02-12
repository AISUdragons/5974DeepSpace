package org.usfirst.frc5974.DeepSpace;

import org.usfirst.frc5974.DeepSpace.Sensors;
import org.usfirst.frc5974.DeepSpace.Controller;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Driving{
    Controller controls = new Controller();
    Sensors sensors = new Sensors();

    //Ports start at 0, not 1.
	VictorSP motorRB = new VictorSP(1); //motor right back
	VictorSP motorRF = new VictorSP(0); //motor right front 
	SpeedControllerGroup motorsRight = new SpeedControllerGroup(motorRF,motorRB);

	VictorSP motorLB = new VictorSP(3); //motor left back
	VictorSP motorLF = new VictorSP(2); //motor left front
	SpeedControllerGroup motorsLeft = new SpeedControllerGroup(motorLF, motorLB);
	
	DifferentialDrive driver = new DifferentialDrive(motorsLeft, motorsRight);

    //Drive Variables
	boolean fastBool = false;		//speed mode: true = fast mode, false = slow mode
	boolean driveNormal = true; 	//drive mode: true = normal tank drive, false = drive straight

    public void tankDrive() {				//left joystick controls left wheels, right joystick controls right wheels
		//Differential Drive solution - much more elegant
		if (fastBool) {
			driver.tankDrive(-controls.joystickLYAxis, -controls.joystickRYAxis);
		} else {
			driver.tankDrive(-.75*controls.joystickLYAxis, -.75*controls.joystickRYAxis);
		}
	}
	public void driveStraight(){
		boolean useFancy = true;
		double turningValue = 0;
		if (useFancy) {
			//ADIS16448 IMU; set useFancy to true to activate.
			turningValue = (sensors.kAngleSetPoint-sensors.yaw) * sensors.kP;
			//turningValue = (kAngleSetPoint-angleX); //Pretty sure we should use yaw or anglex but idk which
		} else {
			//ADXRS450; set useFancy to false to activate.
			turningValue = (sensors.kAngleSetPoint-sensors.angle) * sensors.kP;
		}

		//Invert direction of turn if we are going backwards
		turningValue = Math.copySign(turningValue, controls.joystickLYAxis);

		//Drive.
		if (fastBool) {
			driver.arcadeDrive(-controls.joystickLYAxis, turningValue);
		} else {
			driver.arcadeDrive(-.75*controls.joystickLYAxis,turningValue);
		}
	}
}