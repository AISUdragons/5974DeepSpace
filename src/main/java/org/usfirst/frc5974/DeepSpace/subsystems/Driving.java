package org.usfirst.frc5974.DeepSpace.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.usfirst.frc5974.DeepSpace.Robot;

public class Driving extends Robot{
    //Sensors sensors = new Sensors();
	
	public DifferentialDrive driver = new DifferentialDrive(motorsLeft, motorsRight);

    //Drive Variables
	public boolean fastBool = false;		//speed mode: true = fast mode, false = slow mode
	public boolean driveNormal = true; 	//drive mode: true = normal tank drive, false = drive straight
	double slowModifier = 0.75; //Set slow mode speed

	public void tankDriver(double L, double R) {				//left joystick controls left wheels, right joystick controls right wheels
		if (fastBool) {
			driver.tankDrive(-L, -R);
			//System.out.println(controls.joystickLYAxis);
		
		} else {
			driver.tankDrive(-slowModifier*L, -slowModifier*R);
		}
	}

	/*public void driveStraight(){
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
	*/
}