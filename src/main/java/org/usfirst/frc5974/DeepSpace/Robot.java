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

		if(controls.runOnce(controls.bumperR,controls.pairBumperR)&&targetLevel<3){
			//if bumper R is pressed and target level is less than 3, increase target level
			targetLevel++;
			System.out.println("Target:"+targetLevel+"\nCurrent:"+currentLevel);
	}
	else if(controls.runOnce(controls.bumperL,controls.pairBumperL)&&targetLevel>0){
			//if bumper L is pressed and target level is more than 0, decrease target level
			targetLevel--;
			System.out.println("Target:"+targetLevel+"\nCurrent:"+currentLevel);
	}

	//Kill if lift hits top or bottom limit switches.
	if(controls.switchBottom){
			liftSpeed=Math.max(0,liftSpeed); //We can't just set it to 0, because the limit switch will continue being held down, disabling the motor for the rest of the game.
			//This ensures the lift speed will be positive.
			System.out.println("bottom");
			currentLevel--;
	}
	if(controls.switchTop){
			liftSpeed = Math.min(0,liftSpeed); //See above comments; this ensures lift speed will be negative.
			System.out.println("top");
			currentLevel++;
	}
	
	//Update limit switches for every level
	if(controls.switchL1){ //If the limit switch for L1 is hit:
			System.out.println("1");
			if(currentLevel<1){ //If the current level is less than L1:
					currentLevel++; //Increase current level
			}
			else if(currentLevel>1){ //If the current level is greater than L1:
					currentLevel--; //Decrease current level.
			}
	}
	if(controls.switchL2){ //Same as above.
			System.out.println("2");
			if(currentLevel<2){
					currentLevel++;
			}
			else if(currentLevel>2){
					currentLevel--;
			}
	}
	if(controls.switchL3){
			System.out.println("3");
			if(currentLevel<3){
					currentLevel++;
			}
			else if(currentLevel>3){
					currentLevel--;
			}
	}
	if(targetLevel<currentLevel){
		liftSpeed=-speedModifier; //go down
}
else if(targetLevel>currentLevel){
		liftSpeed=speedModifier; //go up
}
else if(targetLevel==currentLevel){
		liftSpeed=0; //stop
}
		lift.motorLift.set(liftSpeed);
		//robotDrive.driver.tankDrive(controls.joystickLYAxis,controls.joystickRYAxis);
		if(controls.buttonX){
			System.out.println("Target level: "+targetLevel+"\nCurrent level: "+currentLevel);
		}
		robotDrive.tankDriver(controls.joystickLYAxis,controls.joystickRYAxis);

		//Climb
		if(controls.buttonStart){
            climber.motorClimb.set(climber.climbSpeed);
        }
        else if(!controls.buttonStart){
            climber.motorClimb.set(0);
		}

		//Intake stuff
		if(controls.buttonX&&currentLevel==0){
            lift.intake();
        }
        else if(controls.buttonX&&currentLevel>0){
            lift.shoot();
        }
        else if(!controls.buttonX){
            lift.motorGrabLeft.set(0);
            lift.motorGrabRight.set(0);
        }
        //However, this won't work if we're using triggers, or if code is just bad, so here's a fallback.
        if(controls.buttonBack&&!lift.hasBall){
            lift.intake();
            lift.intakeActive=true;
        }
        if(controls.buttonBack&&lift.hasBall){
            lift.shoot();
            lift.shootActive=true;
        }
        if(!controls.buttonBack){
            if(lift.intakeActive){
                lift.motorGrabLeft.set(0);
                lift.motorGrabRight.set(0);
                lift.intakeActive=false;
                lift.hasBall=true;
            }
            if(lift.shootActive){
                lift.motorGrabLeft.set(0);
				lift. motorGrabRight.set(0);
                lift.shootActive=false;
                lift.hasBall=false;
            }
        }
    
		

    }
}