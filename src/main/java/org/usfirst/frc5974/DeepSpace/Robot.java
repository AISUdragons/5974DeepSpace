package org.usfirst.frc5974.DeepSpace;

// Last year's github: https://github.com/AISUMechanicalDragons/FIRSTPowerUp5974
// **If copying/pasting code, it MUST be from there.**

/*CONTROLS

Joystick Y axes: drive

B: fast mode toggle
A: drive straight toggle
Y: Reset gyro

Right trigger: Lift up (user controlled)
Left trigger: Lift down (user controlled)

Optional lift controls (enable in Lift.java): (need limit switches to work correctly)
Right bumper: Lift to higher level (automatic)
Left bumper: Lift to lower level (automatic)
*/

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc5974.DeepSpace.commands.*;
import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc5974.DeepSpace.Sensors;
import org.usfirst.frc5974.DeepSpace.Driving;
import org.usfirst.frc5974.DeepSpace.Controller;
//import org.usfirst.frc5974.DeepSpace.Camera;
import org.usfirst.frc5974.DeepSpace.Lift;
import org.usfirst.frc5974.DeepSpace.Climber;

public class Robot extends TimedRobot { //https://wpilib.screenstepslive.com/s/currentCS/m/cpp/l/241853-choosing-a-base-class

	//Subsystems
	Sensors sensors = new Sensors();
	Driving robotDrive = new Driving();
	Controller controls = new Controller();
	//Camera camera = new Camera();
	Lift lift = new Lift();
	Climber climber = new Climber();
	
	//Sendable chooser on SmartDashboard - we can use this to choose different autonomous options, etc from smartdash
	Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

	Timer timer = new Timer();

	//For tracking iterative updates
	int track = 0;
	int check = 10;

	int targetLevel=0;
	double liftSpeed=0;
	int currentLevel=0;
	double speedModifier=0.5;

	public void update() {					//updates everything
		controls.updateController();

		//Calls updateSensors every 10 updates
		track = (track+1) % check;
		if (track == 0) {
			sensors.updateSensors();
		}

		//toggle checks
		robotDrive.fastBool = controls.toggle(controls.buttonB, robotDrive.fastBool, controls.pairB);	//toggles boolean if button is pressed
		robotDrive.driveNormal = controls.toggle(controls.buttonA, robotDrive.driveNormal, controls.pairA);
		if (controls.runOnce(controls.buttonY, controls.pairY)) {
			sensors.gyroReset(); System.out.println("Gyro Reset");
		}
	}

	public void dashboardOutput() {			//sends and displays data to smart dashboard
		SmartDashboard.putBoolean("Fast Mode", robotDrive.fastBool);
		if (robotDrive.driveNormal) {
			SmartDashboard.putString("Drive mode","Tank");
		} else {
			SmartDashboard.putString("Drive mode","Straight");
		}
		
		//ADIS sensor data
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

		//SmartDashboard.putNumber("Center Thing", camera.centerX);
		//SmartDashboard.putNumber("Temperature: ", sensors.FancyIMU.getTemperature());
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
		robotDrive.motorsLeft.set(1);
		robotDrive.motorsRight.set(1);
		lift.motorLift.set(1);
    }

    @Override
    public void teleopInit() {
        // This line stops auto once teleop starts.
		if (autonomousCommand != null) autonomousCommand.cancel();

		//Rumble controller for half a second
		controls.rumble(0.5);
	}

    @Override
    public void teleopPeriodic() {

		Scheduler.getInstance().run();

		update();
		dashboardOutput();
		/*
		if (robotDrive.driveNormal) {
			robotDrive.tankDrive();
		} else {
			robotDrive.driveStraight();
		}
		*/
		
		if(controls.triggerR<0&&controls.triggerL==0){
            //Move up if right trigger is pressed and left isn't
            lift.motorLift.set(-controls.triggerR*speedModifier); //Input is negative, so neg*neg=pos.
        }
        else if(controls.triggerL>0&&controls.triggerR==0){
            //Move down if left trigger is pressed and right isn't
            lift.motorLift.set(-controls.triggerL*speedModifier);
        }
        else if(controls.triggerL==0&&controls.triggerR==0){
            lift.motorLift.set(0);
		}
		
		/*
		if(controls.triggerR<0&&controls.triggerL==0){
            //Move up if right trigger is pressed and left isn't
            //System.out.println(down);
            lift.motorLift.set(-controls.triggerR*speedModifier);
        }
        else if(controls.triggerL>0&&controls.triggerR==0){
            //Move down if left trigger is pressed and right isn't
            //System.out.println();
            lift.motorLift.set(-controls.triggerL*speedModifier);
        }
        else if(controls.triggerL==0&&controls.triggerR==0){
            lift.motorLift.set(0);
        }*/
		

    }
}