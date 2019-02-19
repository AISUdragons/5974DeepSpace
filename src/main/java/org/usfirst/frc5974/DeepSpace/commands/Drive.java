package org.usfirst.frc5974.DeepSpace.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Drive extends Command{

	public VictorSP motorRF = new VictorSP(0); //motor right front 
	public VictorSP motorRB = new VictorSP(1); //motor right back
	public SpeedControllerGroup motorsRight = new SpeedControllerGroup(motorRF,motorRB); //Groups the right motors together into one object

		public VictorSP motorLF = new VictorSP(2); //motor left front
		public VictorSP motorLB = new VictorSP(3); //motor left back
		public SpeedControllerGroup motorsLeft = new SpeedControllerGroup(motorLF, motorLB); //Groups the left motors together into one object


	public DifferentialDrive driver = new DifferentialDrive(motorsLeft, motorsRight);

    //Drive Variables
	public boolean fastBool = false;		//speed mode: true = fast mode, false = slow mode
	public boolean driveNormal = true; 	//drive mode: true = normal tank drive, false = drive straight
	double slowModifier = 0.75; //Set slow mode speed

	public void initDefaultCommand(){
	}
	
	public void tankDriver(double L, double R) {				//left joystick controls left wheels, right joystick controls right wheels
		if (fastBool) {
			driver.tankDrive(-L, -R);
			//System.out.println(controls.joystickLYAxis);
		
		} else {
			driver.tankDrive(-slowModifier*L, -slowModifier*R);
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
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