package org.usfirst.frc5974.DeepSpace;

// Last year's github: https://github.com/AISUMechanicalDragons/FIRSTPowerUp5974
// **If copying/pasting code, it MUST be from there.**

/*Master todo list once the robot is built:
Carriage:
	Double check PWM ports
	Set grabSpeed
	Test
Driving:
	Double check PWM ports
	Test
Lift:
	Double check PWM port
	Double check DIO ports
	Set liftMode
	Set speedModifier
Robot:

Sensors:

Sucker:
	Double check PWM ports
	Double check DIO port
	Set pivotSpeed
	Set suckSpeed
*/

/*CONTROLS
Drive:
	Joystick Y axes: tank drive
	B: Fast mode toggle
Lift:
	Right trigger: Lift up (user controlled)
	Left trigger: Lift down (user controlled)
	[Limit switch dependent:]
	Right bumper: Lift to higher level (automatic)
	Left bumper: Lift to lower level (automatic)
Carriage:
	Back: Intake or shoot (toggle)
	[Limit switch dependent:]
	A: If lift on floor, suck ball in. If in the air, shoot ball.
Sucker:
	X: Climb tier

Free: Y, Start, dPad. Could also free up lift controls maybe
*/

/*
PWM assignments:
	Driving:
		0 motorRB: Drive: RF
		1 motorRF: Drive: RB
		2 motorLB: Drive: LF
		3 motorLF: Drive: LB
	Lift:
		4 motorLift: Lift: up/down
	Carriage:
		5 motorGrabL: Lift: Intake left
		6 motorGrabR: Lift: Intake right
	Sucker:
		*7 motorsClimberBase: Climber: Pivot left + pivot right
		*8 motorsClimberSpinner: Climber: Spinner left + spinner right
*/

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc5974.DeepSpace.commands.*;
import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc5974.DeepSpace.subsystems.*;

public class Robot extends TimedRobot { //https://wpilib.screenstepslive.com/s/currentCS/m/cpp/l/241853-choosing-a-base-class

	//Subsystems
 	private static Sensors sensors;
	public static Subsystem getSensors(){
		return sensors;
	}

	private static Driving robotDrive;
	public static Subsystem getDriving(){
		return robotDrive;
	}

	private static Controller controls;
	public static Subsystem getControls(){
		return controls;
	}

	private static Camera camera;
	public static Subsystem getCamera(){
		return camera;
	}

	private static Lifter lift;
	public static Subsystem getLift(){
		return lift;
	}

	private static Sucker sucker;
	public static Subsystem getSucker(){
		return sucker;
	}
	
	private static Carriage carriage;
	public static Subsystem getCarriage(){
		return carriage;
	} 


	//PWM motor controllers
	
	//Sendable chooser on SmartDashboard - we can use this to choose different autonomous options, etc from smartdash
	Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

	Timer timer = new Timer();

	//For tracking iterative updates
	int track = 0;
	int check = 10;

	public void update() {					//updates everything
		controls.updateController();

		//Calls updateSensors every 10 updates
		track = (track+1) % check;
		if (track == 0) {
			sensors.updateSensors();
		}

		//toggle checks
		robotDrive.fastBool = controls.toggle(controls.buttonB, robotDrive.fastBool, controls.pairB);	//toggles boolean if button is pressed
	}

	public void dashboardOutput() {			//sends and displays data to smart dashboard
		SmartDashboard.putBoolean("Fast Mode", robotDrive.fastBool);
		if (robotDrive.driveNormal) {
			SmartDashboard.putString("Drive mode","Tank");
		} else {
			SmartDashboard.putString("Drive mode","Straight");
		}
		
		//ADIS sensor data
		/* Tbh this just clutters everything up, so I'm going to comment it out for a bit
		SmartDashboard.putNumber("X acceleration", sensors.accelX);
		SmartDashboard.putNumber("Y acceleration", sensors.accelY);
		SmartDashboard.putNumber("Z acceleration", sensors.accelZ);
		SmartDashboard.putNumber("Angle", sensors.fancyAngle);
		SmartDashboard.putNumber("X angle", sensors.angleX);
		SmartDashboard.putNumber("Y angle", sensors.angleY);
		SmartDashboard.putNumber("Z angle", sensors.angleZ);
		SmartDashboard.putNumber("Pitch", sensors.pitch);
		SmartDashboard.putNumber("Yaw", sensors.yaw);
		SmartDashboard.putNumber("Roll", sensors.roll);
		SmartDashboard.putNumber("Rate", sensors.fancyRate);
		SmartDashboard.putNumber("X rate", sensors.rateX);
		SmartDashboard.putNumber("Y rate", sensors.rateY);
		SmartDashboard.putNumber("Z rate", sensors.rateZ);
		
		SmartDashboard.putNumber("latest time interval", sensors.dt);
		SmartDashboard.putNumber("X velocity", sensors.velX);
		SmartDashboard.putNumber("Y velocity", sensors.velY);
		SmartDashboard.putNumber("Z velocity", sensors.velZ);
		SmartDashboard.putNumber("Gravity Angle from Vertical", sensors.gravAngle);

		SmartDashboard.putNumber("Center Thing", camera.centerX);
		//SmartDashboard.putNumber("Temperature: ", sensors.FancyIMU.getTemperature());
		*/

		SmartDashboard.putNumber("Lift Target Level",lift.targetLevel);
		SmartDashboard.putNumber("Lift Current Level",lift.currentLevel);
	}

    public static OI oi;

    @Override
    public void robotInit() {
        oi = new OI();
		
		//I think this is how we choose different autonomous code options from smartdash
        chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());
		SmartDashboard.putData("Auto mode", chooser);
		
		sensors.sensorInit(); //Calibrates sensors
		robotDrive.driver.setRightSideInverted(true); //Right side is inverted
		//lift.encoder.setDistancePerPulse(1); //Only need to run this if we actually get an encoder working
		
		timer.start();

		//camera.cameraInit(); //Initialize camera/image processing

    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){
		lift.liftSpeed=0;
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = chooser.getSelected();
        // schedule the autonomous command (example)
		if (autonomousCommand != null) autonomousCommand.start();
		//The autonomous comments/psuedocode were here - I took it out, it'll be in the github if you want it back.
    }

    @Override
  public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		//If we want to do anything autonomously, we should probably put it in here.
  }

    @Override
  public void teleopInit() {
    // This line stops auto once teleop starts.
		if (autonomousCommand != null) autonomousCommand.cancel();

		//Rumble controller for half a second
		controls.rumble(0.5);
		camera.cameraInit();
		sucker.setup(); //Set sucker to limit switch to prepare for gameplay
	}

    @Override
  public void teleopPeriodic() {

		Scheduler.getInstance().run();

		update();
		dashboardOutput();

		robotDrive.tankDriver(controls.joystickLYAxis,controls.joystickRYAxis);
		sucker.climb(controls.buttonX); //On buttonX, bring the arm down and start spinning

		//carriage.runCarriage(controls.buttonA, controls.buttonBack); //Operate carriage (intake/ball shooter). Also calls sucker.succ().
		lift.runLift(controls.bumperL,controls.bumperR,controls.triggerL,controls.triggerR,controls.pairBumperL,controls.pairBumperR); //Operate the lift and grabber. Currently based on triggers; change mode in Lift.java.
	}	
}