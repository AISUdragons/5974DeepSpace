package org.usfirst.frc5974.DeepSpace;

// Last year's github: https://github.com/AISUMechanicalDragons/FIRSTPowerUp5974
// **If copying/pasting code, it MUST be from there.**

/*CONTROLS

Joystick Y axes: drive
B: fast mode
A: drive straight

*/

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc5974.DeepSpace.commands.*;

import org.usfirst.frc5974.DeepSpace.ADIS16448_IMU;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

//Camera Stuff
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
/*import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import org.opencv.core.Rect;
import org.usfirst.frc5974.grip.GripPipeline;
import java.util.Set;*/

public class Robot extends TimedRobot { //https://wpilib.screenstepslive.com/s/currentCS/m/cpp/l/241853-choosing-a-base-class

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

	//Ports start at 0, not 1.
	VictorSP motorRB = new VictorSP(1); //motor right back
	VictorSP motorRF = new VictorSP(0); //motor right front 
	SpeedControllerGroup motorsRight = new SpeedControllerGroup(motorRF,motorRB);

	VictorSP motorLB = new VictorSP(3); //motor left back 
	VictorSP motorLF = new VictorSP(2); //motor left front
	SpeedControllerGroup motorsLeft = new SpeedControllerGroup(motorLF, motorLB);
	
	DifferentialDrive driver = new DifferentialDrive(motorsLeft, motorsRight);

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
	boolean fastBool = false;		//speed mode: true = fast mode, false = slow mode
	boolean driveNormal = true; 	//drive mode: true = normal tank drive, false = drive straight

	Timer timer = new Timer();

	//Camera Stuff
	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT =240;
	/*private VisionThread visionThread;
	private double centerX = 0.0;
	private DifferentialDrive driver;
	private final Object imgLock = new Object();
	UsbCamera camera = new UsbCamera(String name, String path)*/

	//Sensor Stuff
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	BuiltInAccelerometer accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
	double xVal;
	double yVal;
	double zVal;
	double angle;
	double rate;
	boolean gyroConnected;

	ADIS16448_IMU FancyIMU = new ADIS16448_IMU();
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
		float firstmove;
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
		}

	public void sensorInit() {
		gyro.calibrate();
		FancyIMU.calibrate();
	}
	public void updateSensors() {
		//ADXRS sensor data
		xVal = accel.getX();
		yVal = accel.getY();
		zVal = accel.getZ();
		angle = gyro.getAngle();
		rate = gyro.getRate();
		gyroConnected = gyro.isConnected();

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
	}

	public boolean checkButton(boolean pressed, boolean toggle, int portNum) {
		//When the button is pushed, once it is released, its toggle is changed
		if (pressed) {
			toggle = !toggle;
			while (pressed) {		//TODO while loops can be problematic in Timed Robot because timing may slip.
									// This is a pretty small amount of code though, so it shouldn't be an issue?
				pressed = controller.getRawButton(portNum);
			}
		}
		return toggle;
	}

	//Set dead zone for joysticks
	public void joystickDeadZone() {
		double deadZoneValue=.16;
		if (joystickLXAxis <=deadZoneValue && joystickLXAxis >= -deadZoneValue) {
			joystickLXAxis = 0;
		} else {
			joystickLXAxis = (joystickLXAxis -deadZoneValue)/(1-deadZoneValue); // We may need to change this.
		} if (joystickLYAxis <=deadZoneValue && joystickLYAxis >= -deadZoneValue) {
			joystickLYAxis = 0;
		} else {
			joystickLYAxis = (joystickLYAxis -deadZoneValue)/(1-deadZoneValue);
		} if (joystickRXAxis <=deadZoneValue && joystickRXAxis >= -deadZoneValue) {
			joystickRXAxis = 0;
		} else {
			joystickRXAxis = (joystickRXAxis -deadZoneValue)/(1-deadZoneValue);
		} if (joystickRYAxis <=deadZoneValue && joystickRYAxis >= -deadZoneValue) {
			joystickRYAxis = 0;
		} else {
			joystickRYAxis = (joystickRYAxis -deadZoneValue)/(1-deadZoneValue);
		}
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
		fastBool = checkButton(buttonB, fastBool, 2);				//toggles boolean if button is pressed
		driveNormal = checkButton(buttonA, driveNormal, 1);
		
		//d-pad/POV updates
		dPad = controller.getPOV(0);		//returns a value {-1,0,45,90,135,180,225,270,315}
	}
	
	public void update() {	//updates everything
		updateController();
		updateSensors();
	}

	public void dashboardOutput() {			//sends and displays data to smart dashboard
		//SmartDashboard.putNumber("Time Remaining", GameTime);
		SmartDashboard.putBoolean("Fast Mode", fastBool);
		if(driveNormal){
			SmartDashboard.putString("Drive mode","Tank");
		} else {
			SmartDashboard.putString("Drive mode","Straight");
		}
		SmartDashboard.putBoolean("Gyro Connected?", gyroConnected);
	}
	public void sensitiveOutput(){ //Displays smartdash data that changes very quickly
		SmartDashboard.putNumber("Old X acceleration", xVal);
		SmartDashboard.putNumber("Old Y acceleration", yVal);
		SmartDashboard.putNumber("Old Z acceleration", zVal);
		SmartDashboard.putNumber("Old angle of robot", angle);
		SmartDashboard.putNumber("Old angular velocity", rate);
		
		SmartDashboard.putNumber("X acceleration", accelX);
		SmartDashboard.putNumber("Y acceleration", accelY);
		SmartDashboard.putNumber("Z acceleration", accelZ);
		SmartDashboard.putNumber("Angle", fancyAngle);
		SmartDashboard.putNumber("X angle", angleX);
		SmartDashboard.putNumber("Y angle", angleY);
		SmartDashboard.putNumber("Z angle", angleZ);
		SmartDashboard.putNumber("Pitch", pitch);
		SmartDashboard.putNumber("Yaw", yaw);
		SmartDashboard.putNumber("Roll", roll);
		SmartDashboard.putNumber("Rate", fancyRate);
		SmartDashboard.putNumber("X rate", rateX);
		SmartDashboard.putNumber("Y rate", rateY);
		SmartDashboard.putNumber("Z rate", rateZ);
	}

	public void tankDrive() {	//left joystick controls left wheels, right joystick controls right wheels
		
		//Differential Drive solution - much more elegant
		if(fastBool){
			driver.tankDrive(joystickLYAxis,joystickRYAxis);
		} else{
			driver.tankDrive(joystickLYAxis/2,joystickRYAxis/2);
		}
		
		/* Last year's solution
		if (fastBool) {
			motorRB.set(joystickRYAxis);
			motorRF.set(joystickRYAxis);
			motorLB.set(-joystickLYAxis); //these two are inverted
			motorLF.set(-joystickLYAxis);
		} else {
			motorRB.set(joystickRYAxis/2);
			motorRF.set(joystickRYAxis/2);
			motorLB.set(-joystickLYAxis/2);
			motorLF.set(-joystickLYAxis/2);
		}
		*/
	}
	
	//This is a code example from https://wiki.analog.com/first/adis16448_imu_frc/java.
	private static final double kAngleSetPoint=0.0; //straight ahead
	private static final double kP=0.005; //proportional turning constant. not sure what this is, ngl

	//gyro calibration constant, may need to be adjusted. 360 is set to correspond to one full revolution.
	private static final double kVoltsPerDegreePerSecond=0.0128;

	public void driveStraight() {
		boolean useFancy = true;
		double turningValue = 0;
		if (useFancy) {
			//ADIS16448 IMU; set useFancy to true to activate.
			turningValue = (kAngleSetPoint-yaw) * kP;
			//turningValue = (kAngleSetPoint-angleX); //Pretty sure we should use yaw or anglex but idk which
		} else {
			//ADXRS450; set useFancy to false to activate.
			turningValue = (kAngleSetPoint-angle) * kP;
		}

		//Invert direction of turn if we are going backwards
		turningValue = Math.copySign(turningValue, joystickLYAxis);

		//Drive.
		if (fastBool) {
			driver.arcadeDrive(joystickLYAxis, turningValue);
		} else {
			driver.arcadeDrive(joystickLYAxis/2, turningValue);
		}
	}

    public static OI oi;

    @Override
    public void robotInit() {
        oi = new OI();
		
		//I think this is how we choose different autonomous code options from smartdash
        chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());
		SmartDashboard.putData("Auto mode", chooser);
		
		sensorInit(); //Calibrates sensors
		driver.setRightSideInverted(true);
		
		//Camera Stuff
		new Thread(() -> {
			//Creates a UsbCamera on the default port and streams output on MjpegServer [1]
			//equivalent to "".startAutomaticCapture(0), which is equivalent to "".startAutomaticCapture("USB Camera 0", 0)
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

			//Creates an image input (sink) that takes video from the primary feed (UsbCamera camera)
			//equivalent to "".getVideo(camera)
			CvSink cvSink = CameraServer.getInstance().getVideo();

			//Creates an image stream (source) MjpegServer [2] with the name "Blur"
			CvSource outputStream = CameraServer.getInstance().putVideo("Blur", IMG_WIDTH, IMG_HEIGHT);
			
			Mat source = new Mat(); //unreleated to CvSource
			Mat output = new Mat();

			while(!Thread.interrupted()) {
				//Applies the 'Imgproc.COLOR_BGR2GRAY' filter to a frame from 'cvSink' and puts the result on 'outputStream'
				cvSink.grabFrame(source);
				Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
				outputStream.putFrame(output);
			}
		}).start();
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
		controller.setRumble(Joystick.RumbleType.kRightRumble, 0.5);
		controller.setRumble(Joystick.RumbleType.kLeftRumble, 0.5);
		Timer.delay(0.5);
		controller.setRumble(Joystick.RumbleType.kRightRumble, 0);
		controller.setRumble(Joystick.RumbleType.kLeftRumble, 0);
		
		timer.start();
	}

    @Override
    public void teleopPeriodic() {
		Scheduler.getInstance().run();
		update();
		if(Math.abs(Math.round(timer.get())-timer.get())<.01){ //If the timer is within .01 of a whole second, run sensitive output.
			sensitiveOutput();
		}
		dashboardOutput();
		if(driveNormal){
			tankDrive();
		} else{
			driveStraight();
		}
    }
}