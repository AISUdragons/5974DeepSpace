package org.usfirst.frc5974.DeepSpace;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Driving{
    Controller controls = new Controller();
    Sensors sensors = new Sensors();

    //Ports start at 0, not 1.
	VictorSP motorRB = new VictorSP(1); //motor right back
	VictorSP motorRF = new VictorSP(0); //motor right front 
	SpeedControllerGroup motorsRight = new SpeedControllerGroup(motorRF,motorRB); //Groups the right motors together into one object

	VictorSP motorLB = new VictorSP(3); //motor left back
	VictorSP motorLF = new VictorSP(2); //motor left front
	SpeedControllerGroup motorsLeft = new SpeedControllerGroup(motorLF, motorLB); //Groups the left motors together into one object
	
	DifferentialDrive driver = new DifferentialDrive(motorsLeft, motorsRight);

    //Drive Variables
	boolean fastBool = false;		//speed mode: true = fast mode, false = slow mode
	boolean driveNormal = true; 	//drive mode: true = normal tank drive, false = drive straight
	double slowModifier = 0.75; //Set slow mode speed

    public void tankDrive() {				//left joystick controls left wheels, right joystick controls right wheels
		if (fastBool) {
			driver.tankDrive(-controls.joystickLYAxis, -controls.joystickRYAxis);
		} else {
			driver.tankDrive(-slowModifier*controls.joystickLYAxis, -slowModifier*controls.joystickRYAxis);
		}
	}

	public void driveStraight(){
		double turningValue = 0;
		
		turningValue = (sensors.kAngleSetPoint-sensors.yaw) * sensors.kP;

		//Invert direction of turn if we are going backwards
		turningValue = Math.copySign(turningValue, controls.joystickLYAxis);

		//Drive.
		if (fastBool) {
			driver.arcadeDrive(-controls.joystickLYAxis, turningValue);
		} else {
			driver.arcadeDrive(-slowModifier*controls.joystickLYAxis,turningValue);
		}
	}
}