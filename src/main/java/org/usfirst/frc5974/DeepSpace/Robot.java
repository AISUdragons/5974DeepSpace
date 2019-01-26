// I will be here forever. You do like lies, right?
// Last year's github: https://github.com/AISUMechanicalDragons/FIRSTPowerUp5974
// **If copying/pasting code, it MUST be from there.**

// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5974.DeepSpace;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc5974.DeepSpace.commands.*;

//IDK if we need all these imports
//import org.usfirst.frc5974.DeepSpace.subsystems.*;
import edu.wpi.first.wpilibj.*;         //Imports a lot of stuff (motors, controllers, timer, etc.)

//Camera Stuff
import edu.wpi.first.cameraserver.CameraServer;
/*import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.UsbCamera;
import org.usfirst.frc5974.grip.GripPipeline;
//import java.util.Set;*/

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot { //https://wpilib.screenstepslive.com/s/currentCS/m/cpp/l/241853-choosing-a-base-class

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

	//Ports start at 0, not 1.
	//TODO: Set up motors once we know ports/inversion
	VictorSP motorRB = new VictorSP(0); //motor right back
	VictorSP motorRF = new VictorSP(1); //motor right front 
	VictorSP motorLB = new VictorSP(3); //motor left back 
	VictorSP motorLF = new VictorSP(2); //motor left front
	
	//Variables for the Controller
	Joystick controller = new Joystick(0);	//controller
	double joystickLXAxis;			//left joystick x-axis
	double joystickLYAxis;			//left joystick y-axis
	double joystickRXAxis;			//right joystick x-axis
	double joystickRYAxis;			//right joystick y-axis
	double triggerL;				//left trigger
	double triggerR;				//right trigger
	boolean bumperL;				//left bumper
	boolean bumperR;				//right bumper
	boolean buttonX;				//x button
	boolean buttonY;				//y button
	boolean buttonA;				//a button
	boolean buttonB;				//b button
	int dPad;					    //d-pad
	boolean joystickLPress;		    //left joystick button press
	boolean joystickRPress;		    //right joystick button press
	boolean buttonStart;			//start button
	boolean buttonBack;			    //back button
	
	//Drive Variables
	boolean tankDriveBool = true;	//drive mode: true = tank drive, false = arcade drive
	boolean fastBool = false;		//speed mode: true = fast mode, false = slow mode

	//time variables [see updateTimer()]
	//Not sure if we'll actually use these, but here they are.
	Timer timer = new Timer();
	Timer timerTest = new Timer();
	double dT = 0; //time difference (t1-t0)
	double t0 = 0; //time start
	double t1 = 0; //time end

	//Camera Stuff
	/*private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT =240;
	private VisionThread visionThread;
	private double centerX = 0.0;
	private RobotDrive drive;
	private final Object imgLock = new Object();
	//UsbCamera camera = new UsbCamera(String name, String path)*/

	//Sensor Stuff
	//This is for the little FRC Gyro/Accel chip on the top right corner of the RIO. 
	//The thicc ADIS sensor in the middle needs different libraries and stuff, but that'd be fun to play with too.
	private Accelerometer accel; 
	double xVal;
	double yVal;
	double zVal;
	private Gyro gyro;
	double angle;
	double gyroRate;
	public void sensorInit() {
		gyro = new AnalogGyro(0);
		gyro.calibrate(); //not actually sure what this does, but it seemed like a good idea.
		//accel = new BuiltInAccelerometer();
		accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
	}
	public void updateSensors() {
		xVal = accel.getX();
		yVal = accel.getY();
		zVal = accel.getZ();
		gyroRate = gyro.getRate(); //rate of rotation of gyro
		angle = gyro.getAngle(); //Goes from 360 to 361 degrees. I think .reset() is maybe a thing though idk
	}

	public boolean checkButton(boolean pressed, boolean toggle, int portNum) {
		//When the button is pushed, once it is released, its toggle is changed
		if (pressed) {
			toggle = !toggle;
			while (pressed) {		//TODO while loops can be problematic in Timed Robot because timing may slip.
									// This is a pretty small amount of code though, so it shouldn't be an issue. 
				pressed = controller.getRawButton(portNum);
			}
		}
		return toggle;
	}

	//Set dead zone for joysticks
	//TODO: May need some testing/fine-tuning
	//does this need to be a seperate function?
	public void joystickDeadZone() {
		if (joystickLXAxis <= 0.075 && joystickLXAxis >= -0.075) {
			joystickLXAxis = 0;
		} if (joystickLYAxis <= 0.075 && joystickLYAxis >= -0.075) {
			joystickLYAxis = 0;
		}
		if (joystickRXAxis <= 0.075 && joystickRXAxis >= -0.075) {
			joystickRXAxis = 0;
		} if (joystickRYAxis <= 0.075 && joystickRYAxis >= -0.075) {
			joystickRYAxis = 0;
		}
	}

	public void updateTimer() {	//sets change in time between the current running of a periodic function and the previous running
		t0 = t1;
		t1 = timer.get();
		dT = t1 - t0;
	}

	public void updateController() {		//updates all controller features
		//joystick updates
		joystickLXAxis = controller.getRawAxis(0);		//returns a value [-1,1]
		joystickLYAxis = controller.getRawAxis(1);		//returns a value [-1,1]
		joystickRXAxis = controller.getRawAxis(4);		//returns a value [-1,1]
		joystickRYAxis = controller.getRawAxis(5);		//returns a value [-1,1]
		joystickLPress = controller.getRawButton(9);	//returns a value {0,1}
		joystickRPress = controller.getRawButton(10);	//returns a value {0,1}
        joystickDeadZone();

		//trigger updates
		triggerL = controller.getRawAxis(2);		//returns a value [0,1]
		triggerR = controller.getRawAxis(3);		//returns a value [0,1]
		
		//bumper updates
		bumperL = controller.getRawButton(5);		//returns a value {0,1}
		bumperR = controller.getRawButton(6);		//returns a value {0,1}
		
		//button updates
		buttonX = controller.getRawButton(3);		//returns a value {0,1}
		buttonY = controller.getRawButton(4);		//returns a value {0,1}
		buttonA = controller.getRawButton(1);		//returns a value {0,1}
		buttonB = controller.getRawButton(2);		//returns a value {0,1}
		
		buttonBack = controller.getRawButton(7);	//returns a value {0,1}
		buttonStart = controller.getRawButton(8);	//returns a value {0,1}
		
		//toggle checks
		tankDriveBool = checkButton(buttonX, tankDriveBool, 3);		//toggles boolean if button is pressed
		fastBool = checkButton(buttonB, fastBool, 2);				//toggles boolean if button is pressed
		
		//d-pad/POV updates
		dPad = controller.getPOV(0);		//returns a value {-1,0,45,90,135,180,225,270,315}

		//d-pad/POV turns
		//??
	}
	
	public void update() {	//updates everything
		updateController();
		updateTimer();
		updateSensors();
	}

	public void dashboardOutput() {			//sends and displays data to smart dashboard
		//SmartDashboard.putNumber("Time Remaining", GameTime);
		SmartDashboard.putBoolean("Tank Drive Style", tankDriveBool);
		SmartDashboard.putBoolean("Fast Mode", fastBool);
		SmartDashboard.putNumber("Team Number", 5974);
		SmartDashboard.putNumber("X Value",xVal);
		SmartDashboard.putNumber("Y Value",yVal);
		SmartDashboard.putNumber("Z Value",zVal);
		SmartDashboard.putNumber("Angle",angle);
		SmartDashboard.putNumber("Rate",gyroRate);
	}

	public void tankDrive() {	//left joystick controls left wheels, right joystick controls right wheels
		if (fastBool) {
			motorRB.set(joystickRYAxis);
			motorRF.set(joystickRYAxis);
			//TODO: Invert the correct motors
			motorLB.set(-joystickLYAxis); //Assuming these two are inverted.
			motorLF.set(-joystickLYAxis);
		} else {
			motorRB.set(joystickRYAxis/2);
			motorRF.set(joystickRYAxis/2);
			motorLB.set(-joystickLYAxis/2);
			motorLF.set(-joystickLYAxis/2);

		}
	}
	
	public void arcadeDrive() {	//arcade drive: left joystick controls all driving
		//right wheels have more forward power the farther left the joystick is pushed
		//left wheels have more forward power the farther right the joystick is pushed
		//All wheels have more forward power the farther up the joystick is pushed
		//X-axis input is halved
		if (fastBool) {
			motorRB.set((joystickLYAxis + joystickLXAxis/2));
			motorRF.set((joystickLYAxis + joystickLXAxis/2));
			//TODO: Invert the correct motors
			motorLB.set(-(joystickLYAxis - joystickLXAxis/2));
			motorLF.set(-(joystickLYAxis - joystickLXAxis/2));
		} else {
			motorRB.set((joystickLYAxis + joystickLXAxis/2)/2);
			motorRF.set((joystickLYAxis + joystickLXAxis/2)/2);
			motorLB.set(-(joystickLYAxis - joystickLXAxis/2)/2);
			motorLF.set(-(joystickLYAxis - joystickLXAxis/2)/2);
		}
}
    public static OI oi;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
		//Add in any needed initialization with sensors, autonomous/sendable chooser, etc.

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();

        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
		SmartDashboard.putData("Auto mode", chooser);
		
		//Camera Stuff
		//UsbCamera camera = 
		CameraServer.getInstance().startAutomaticCapture();	
		sensorInit(); //Initiate FRC gyro/accel (NOT ADIS)
		//camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		/*visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
			if (!pipeline.filterContoursOutput.isEmpty()) {
				Rect r = Imgproc.boundingRect(pipeline.filterContourOutput().get(0));
				synchronized (imgLock) {
					centerX = r.x + (r.width / 2);
				}
			}
		});
		visionThread.start();
		//drive = new RobotDrive(1, 2);*/

		/*NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable table = inst.getTable("GRIP/myContours Report");
		double[] defaultValue = new double[0];
		while(true) {
			double[] areas = table.getEntry("area").getDoubleArray(defaultValue);
			System.out.print("areas: ");
			for (double area : areas) {
				System.out.print(area + " ");
			}
			System.out.println();
			Timer.delay(1);
		}*/
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
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
		if (autonomousCommand != null) autonomousCommand.cancel();

		//Rumble controller for half a second
		controller.setRumble(Joystick.RumbleType.kRightRumble, 0.5);
		controller.setRumble(Joystick.RumbleType.kLeftRumble, 0.5);
		Timer.delay(0.5);
		controller.setRumble(Joystick.RumbleType.kRightRumble, 0);
		controller.setRumble(Joystick.RumbleType.kLeftRumble, 0);
		
		timer.start();
	}
    
    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
		Scheduler.getInstance().run();

		update();
		dashboardOutput();

		if (tankDriveBool) {
			tankDrive();
		} else {
			arcadeDrive();
		}
    }
}