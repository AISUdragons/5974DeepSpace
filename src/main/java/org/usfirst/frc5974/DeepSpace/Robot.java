package org.usfirst.frc5974.DeepSpace;


// Last year's github: https://github.com/AISUMechanicalDragons/FIRSTPowerUp5974
// **If copying/pasting code, it MUST be from there.**

/*CONTROLS

Joystick Y axes: drive
B: fast mode
A: drive straight
Y: Reset gyro

*/

import org.usfirst.frc5974.DeepSpace.Sensors;
import org.usfirst.frc5974.DeepSpace.Driving;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc5974.DeepSpace.commands.*;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.VictorSP;


public class Robot extends TimedRobot { //https://wpilib.screenstepslive.com/s/currentCS/m/cpp/l/241853-choosing-a-base-class

	Sensors sensors = new Sensors();
	Driving robotDrive = new Driving();
	Controller controls = new Controller();
	Camera camera = new Camera();
	
	Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

	VictorSP motorLift = new VictorSP(4); // lift motor

	Timer timer = new Timer();
	int track = 0;
	int check = 10;
	
	/*
	if(motorLift.get()==0&&buttonA){
		double t0=timer.get();
		motorLift.set(1)
	}
	if(timer.get()-t0>1){
		motorLift.set(0);
	}
	*/

	public void update() {					//updates everything
		controls.updateController();

		//Calls updateSensors every 10 updates
		track = (track+1) % check;
		if (track == 0) {
			sensors.updateSensors();
		}
	}

	public void dashboardOutput() {			//sends and displays data to smart dashboard
		//SmartDashboard.putNumber("Time Remaining", GameTime);
		SmartDashboard.putBoolean("Fast Mode", robotDrive.fastBool);
		if (robotDrive.driveNormal) {
			SmartDashboard.putString("Drive mode","Tank");
		} else {
			SmartDashboard.putString("Drive mode","Straight");
		}

		SmartDashboard.putNumber("Left Joystick Y:",controls.joystickLYAxis);
		SmartDashboard.putNumber("Right Joystick Y:",controls.joystickRYAxis);

		SmartDashboard.putBoolean("Old Gyro Connected?", sensors.gyroConnected);

		//Sensor data
		SmartDashboard.putNumber("Old X acceleration", sensors.xVal);
		SmartDashboard.putNumber("Old Y acceleration", sensors.yVal);
		SmartDashboard.putNumber("Old Z acceleration", sensors.zVal);
		SmartDashboard.putNumber("Old angle of robot", sensors.angle);
		SmartDashboard.putNumber("Old angular velocity", sensors.rate);
		
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
		SmartDashboard.putNumber("Center Thing", camera.centerX);
		SmartDashboard.putNumber("Temperature: ", sensors.FancyIMU.getTemperature());
	}
	// start of lift proto (?) code. Will probably need changes.
	Timer liftTimer = new Timer();
	public void liftUp() {
		if (controls.buttonX){
			motorLift.set(1);
			//timer.delay(0.5); // set 0.5 to whatever is necessary to get the lift to the correct height.
		}
	}
	public void liftDown() {
		motorLift.set(-1);
		//timer.delay(0.5); // set 0.5 to whatever is necessary to get the lift to the correct height.
	}

    public static OI oi;

    @Override
    public void robotInit() {
        oi = new OI();
		
		//I think this is how we choose different autonomous code options from smartdash
        chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());
		SmartDashboard.putData("Auto mode", chooser);
		
		sensors.sensorInit(); //Calibrates sensors
		robotDrive.driver.setRightSideInverted(true);
		
		timer.start();

		camera.cameraInit();

    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){

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
		/*
		The placement of the following section of code may be wrong, but it seems to work here. Also, the plan for autonomous movement is purely a first draft.

		I plan for it to grab a ball from the depot, then put it in the front right slot of the cargo ship. Depending on how long this takes, I may have it grab another
		cargo ball and put it into the slot right next to it. However, as of now, I'm not sure how fast it can move and grab/deposit cargo. The robot will probably start
		on the second tier, on the right platform when facing the glass from the field. A primitive map, with * being a filled cargo slot and 0 being an empty one.
		The numbers are the platforms.
		|     1 2
		|000  1 3
		|0**  1 3
		|     1 2 <-- our robot
		*/
		/*float firstmove;
		float turntime1;
		float secondmove;
		float turntime2;
		float thirdmove;
		float turntime3;
		float turnaround;
		public void Autonomous() {
			//get down from platform two. assume I'm facing forward just in front of the plaform
			motorLB.set(1);
			motorRB.set(1);
			motorLF.set(1);
			motorRF.set(1);

			timer.delay(firstmove); // I'm not completely sure what Java means by the green underline on the timer.delay(); lines.
			motorLB.set(0);
			motorLF.set(0);
			timer.delay(turntime1);
			motorLB.set(1);
			motorRB.set(1);
			timer.delay(secondmove);
			motorLB.set(0);
			motorLF.set(0);
			motorRB.set(0);
			motorRF.set(0);
			//grab the cargo ball
			motorLB.set(1);
			motorRB.set(-1);
			motorRF.set(-1);
			motorLF.set(1);
			timer.delay(turnaround);
			motorLB.set(1);
			motorRB.set(1);
			motorRF.set(1);
			motorLF.set(1);
			timer.delay(thirdmove);
			motorLB.set(1);
			motorRB.set(-1);
			motorRF.set(-1);
			motorLF.set(1);
			timer.delay(turntime3);
			motorLB.set(0);
			motorRB.set(0);
			motorRF.set(0);
			motorLF.set(0);
			//put the ball in the cargo slot
			motorLB.set(1);
			motorRB.set(-1);
			motorRF.set(-1);
			motorLF.set(1);
			timer.delay(turnaround);
			motorLB.set(1);
			motorRB.set(1);
			motorRF.set(1);
			motorLF.set(1);
			timer.delay(thirdmove);
			motorLB.set(1);
			motorRB.set(-1);
			motorRF.set(-1);
			motorLF.set(1);
			timer.delay(turntime3);
			motorLB.set(0);
			motorRB.set(0);
			motorRF.set(0);
			motorLF.set(0);
			// grab another cargo ball
			motorLB.set(1);
			motorRB.set(-1);
			motorRF.set(-1);
			motorLF.set(1);
			timer.delay(turnaround);
			motorLB.set(1);
			motorRB.set(1);
			motorRF.set(1);
			motorLF.set(1);
			timer.delay(thirdmove);
			motorLB.set(1);
			motorRB.set(-1);
			motorRF.set(-1);
			motorLF.set(1);
			timer.delay(turntime3);
			motorLB.set(0);
			motorRB.set(0);
			motorRF.set(0);
			motorLF.set(0);
			// put it in again. this may be all the time we have
		}*/
		//Autonomous();
    }

    @Override
    public void autonomousPeriodic() {
		Scheduler.getInstance().run();
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
		double t0=0;
		if(robotDrive.motorRB.get()==0&&controls.buttonA){
			t0=timer.get();
			robotDrive.motorRB.set(1);
		}
		if(timer.get()-t0>1){
			robotDrive.motorRB.set(0);
		}

		Scheduler.getInstance().run();
		update();
		dashboardOutput();
		if (robotDrive.driveNormal) {
			robotDrive.tankDrive();
		} else {
			robotDrive.driveStraight();
		}
    }
}