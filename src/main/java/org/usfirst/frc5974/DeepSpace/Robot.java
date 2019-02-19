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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc5974.DeepSpace.commands.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

import org.usfirst.frc5974.DeepSpace.subsystems.*;

import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

/*import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionRunner;*/

import org.opencv.core.Rect;
import org.usfirst.frc5974.grip.GripPipeline;
//import java.util.Set;

public class Robot extends TimedRobot { //https://wpilib.screenstepslive.com/s/currentCS/m/cpp/l/241853-choosing-a-base-class
	//Lift
	double grabSpeed = 1; //grabber/intake motor speed
    boolean hasBall = false;
    boolean intakeActive=false;
		boolean shootActive=false;
		double pivotSpeed;
    double suckSpeed=.5;
    double climbSpeed=.75;

    		//Sucker
		//These will have 2 motor controllers each.
		public VictorSP motorsSuckerBase = new VictorSP(7); 
		public VictorSP motorsSuckerSpinner = new VictorSP(8);
		
		//Sucker
		public DigitalInput switchSucker = new DigitalInput(5);
    public DigitalInput switchBase = new DigitalInput(6);


    //Carriage
	public VictorSP motorGrabL = new VictorSP(5);
	public VictorSP motorGrabR = new VictorSP(6);
    Controller controls = new Controller();
    //Variables
    public double speedModifier = .5; //change this to make lift move faster or slower
    public int targetLevel = 0; //level it's supposed to go to
    public double currentLevel = 0; //level it's currently at
    public double liftSpeed = 0; //the value we set motorLift to
    public int liftMode = 0; //0 is trigger, 1 is limit switch.

    //Lift
		public DigitalInput switchBottom = new DigitalInput(0); //TODO: Set limit switches to the correct ports
        public DigitalInput switchL1 = new DigitalInput(1);
        public DigitalInput switchL2 = new DigitalInput(2);
        public DigitalInput switchL3 = new DigitalInput(3);
        public DigitalInput switchTop = new DigitalInput(4);

        public VictorSP motorLift = new VictorSP(4);
				
				
	//Driving
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
	
	//Controller
	    //Variables for the Controller
			Joystick controller = new Joystick(0);	//controller
			public double joystickLXAxis;			//left joystick x-axis
			public double joystickLYAxis;			//left joystick y-axis
			public double joystickRXAxis;			//right joystick x-axis
			public double joystickRYAxis;			//right joystick y-axis
			public double triggerL;				//left trigger
			public double triggerR;				//right trigger
			public boolean bumperL;				//left bumper
			public boolean bumperR;				//right bumper
			public boolean buttonX;				//x button
			public boolean buttonY;				//y button
			public boolean buttonA;				//a button
			public boolean buttonB;				//b button
			int dPad;					    //d-pad
			public boolean joystickLPress;		    //left joystick button press
			public boolean joystickRPress;		    //right joystick button press
			public boolean buttonStart;			//start button
			public boolean buttonBack;			    //back button
			
			public boolean[] pairX = {false, false};
			public boolean[] pairY = {false, false};
			public boolean[] pairA = {false, false};
			public boolean[] pairB = {false, false};
			public boolean[] pairBumperR = {false,false};
			public boolean[] pairBumperL = {false,false};
				
			boolean pressed = false;
			
			public boolean toggle(boolean button, boolean toggle, boolean[] buttonPair) {
				return runOnce(button, buttonPair) ? !toggle : toggle;
			}
		
			public boolean checkButton(boolean pressed, boolean toggle, int portNum) {
				//When the button is pushed, once it is released, its toggle is changed
				if (pressed) {
					toggle = !toggle;
					while (pressed) {		//While loops can be problematic in Timed Robot because timing may slip.
											// This is a pretty small amount of code though, so it shouldn't be an issue?
						pressed = controller.getRawButton(portNum);
					}
				}
				return toggle;
			}
		
			public boolean runOnce(boolean pressed, boolean[] buttonPair) {	//first in buttonPair is run, second is completed
				boolean completed = buttonPair[0];
				if (pressed && !completed) {
					buttonPair[0] = true;
					buttonPair[1] = true;
				} else {
					if (!pressed && completed) {
						buttonPair[0] = false;
					}
					if (buttonPair[1]) {buttonPair[1] = false;}
				}
				return buttonPair[1];
			}
		
			public void joystickDeadZone() {		//Set dead zone for joysticks
				double deadZoneValue=.16;
				if (joystickLXAxis <=deadZoneValue && joystickLXAxis >= -deadZoneValue) {
					joystickLXAxis = 0;
				}
				if (joystickLYAxis <=deadZoneValue && joystickLYAxis >= -deadZoneValue) {
					joystickLYAxis = 0;
				}
				if (joystickRXAxis <=deadZoneValue && joystickRXAxis >= -deadZoneValue) {
					joystickRXAxis = 0;
				} 
				if (joystickRYAxis <=deadZoneValue && joystickRYAxis >= -deadZoneValue) {
					joystickRYAxis = 0;
				} 
		
				if(triggerL<=0.0001){
					triggerL=0;
				}
				if(triggerR>=-0.0001){
					triggerR=0;
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
				//trigger updates
				triggerL = controller.getRawAxis(2);		//returns a value [0,1]
				triggerR = controller.getRawAxis(3);		//returns a value [0,1]
				joystickDeadZone();
				//System.out.println(joystickLYAxis);
				
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
				
				//d-pad/POV updates
				dPad = controller.getPOV(0);		//returns a value {-1,0,45,90,135,180,225,270,315}
			}
				
				public void rumble(double duration){
						controller.setRumble(Joystick.RumbleType.kRightRumble, 0.5);
				controller.setRumble(Joystick.RumbleType.kLeftRumble, 0.5);
				Timer.delay(duration);
				controller.setRumble(Joystick.RumbleType.kRightRumble, 0);
				controller.setRumble(Joystick.RumbleType.kLeftRumble, 0);
				}

			//Sensors
			//ADXRS450_Gyro gyro = new ADXRS450_Gyro();
			//BuiltInAccelerometer accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
			Timer timer2 = new Timer();
		double xVal;
		double yVal;
		double zVal;
		double angle;
		double rate;
			boolean gyroConnected;
			
			//This is a code example from https://wiki.analog.com/first/adis16448_imu_frc/java.
		public static final double kAngleSetPoint = 0.0; //straight ahead
		public static final double kP = 0.005; //proportional turning constant. not sure what this is, ngl
	
		//gyro calibration constant, may need to be adjusted. 360 is set to correspond to one full revolution.
		//private static final double kVoltsPerDegreePerSecond=0.0128;
	
		public static final ADIS16448_IMU FancyIMU = new ADIS16448_IMU();
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
	
		double velX;
		double velY;
		double velZ;
		double time;
		double prevTime = 0;
		double dt;
	
		double gravAngle;
		
		public void gyroReset() { //it resets the gyro
			FancyIMU.reset();
			//gyro.reset();
		} 
	
		public void sensorInit() {
			//gyro.calibrate();
			FancyIMU.calibrate();
			velX = velY = velZ = 0;
		}
		public void updateSensors() {
			//ADXRS sensor data
			/*xVal = accel.getX();
			yVal = accel.getY();
			zVal = accel.getZ();
			angle = gyro.getAngle();
			rate = gyro.getRate();
			gyroConnected = gyro.isConnected();*/
	
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
	
			time = timer.get();
			dt = time - prevTime;
			velX += accelX * dt;
			velY += accelY * dt;
			velZ += accelZ * dt;
			prevTime = time;
	
			gravAngle = Math.acos(accelX) * 180/Math.PI;
		}
	
	//Sendable chooser on SmartDashboard - we can use this to choose different autonomous options, etc from smartdash
	Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

	Timer timer = new Timer();

	//For tracking iterative updates
	int track = 0;
	int check = 10;

	public void update() {					//updates everything
		updateController();

		//Calls updateSensors every 10 updates
		track = (track+1) % check;
		if (track == 0) {
			updateSensors();
		}

		//toggle checks
		fastBool = toggle(buttonB, fastBool, pairB);	//toggles boolean if button is pressed
	}

	public void dashboardOutput() {			//sends and displays data to smart dashboard
		SmartDashboard.putBoolean("Fast Mode", fastBool);
		
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

		SmartDashboard.putNumber("Lift Target Level",targetLevel);
		SmartDashboard.putNumber("Lift Current Level",currentLevel);
	}

    public static OI oi;

    @Override
    public void robotInit() {
        oi = new OI();
		
		//I think this is how we choose different autonomous code options from smartdash
        chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());
		SmartDashboard.putData("Auto mode", chooser);
		
		sensorInit(); //Calibrates sensors
		driver.setRightSideInverted(true); //Right side is inverted
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
		liftSpeed=0;
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
		rumble(0.5);
		cameraInit();
		setup(); //Set sucker to limit switch to prepare for gameplay
	}
	public void intake(){
		motorGrabL.set(grabSpeed);
		motorGrabR.set(-grabSpeed);
		succ();
}
public void succ(){
	motorsSuckerSpinner.set(suckSpeed);
}
private static final int IMG_WIDTH = 240;
	private static final int IMG_HEIGHT = 180;
	private static final int fps = 20;
	private VisionThread visionThread;
	double centerX = 0.0;
	private final Object imgLock = new Object();
    //UsbCamera camera = new UsbCamera(String name, String path)*/
    
    //Camera Stuff
    public void cameraInit(){
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		camera.setFPS(fps);

		new Thread(() -> {
			//Creates a UsbCamera on the default port and streams output on MjpegServer [1]
			//equivalent to "".startAutomaticCapture(0), which is equivalent to "".startAutomaticCapture("USB Camera 0", 0)

			//Creates an image input (sink) that takes video from the primary feed (UsbCamera camera)
			//equivalent to "".getVideo(camera)
			CvSink cvSink = CameraServer.getInstance().getVideo();

			//Creates an image stream (source) MjpegServer [2] with the name "Blur"
			CvSource outputStream = CameraServer.getInstance().putVideo("Blur", IMG_WIDTH, IMG_HEIGHT);
			
			Mat source = new Mat(); //unreleated to CvSource
			Mat output = new Mat();

			while (!Thread.interrupted()) {
				//Applies the 'Imgproc.COLOR_BGR2GRAY' filter to a frame from 'cvSink' and puts the result on 'outputStream'
				cvSink.grabFrame(source);
				Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
				outputStream.putFrame(output);
			}
		}).start();

		visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
			if (!pipeline.findBlobsOutput().empty()) {
				Rect r = Imgproc.boundingRect(pipeline.findBlobsOutput());
				synchronized (imgLock) {
					centerX = r.x + (r.width / 2);
				}
			}
		});
		visionThread.start();

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

public void setup(){

	motorsSuckerBase.set(pivotSpeed);
	if(switchSucker.get()){
			motorsSuckerBase.set(0);
	}
	
}

public void climb(boolean X){
	
	if(X){
			if(switchBase.get()){
					motorsSuckerBase.set(0);
			}
			else{
					motorsSuckerBase.set(pivotSpeed);
			}
			motorsSuckerSpinner.set(climbSpeed);
	}
	
}

public void shoot(){
		motorGrabL.set(-grabSpeed);
		motorGrabR.set(grabSpeed);
}
	public void runCarriage(boolean a, boolean back){
		//Theoretically, this will take in balls if it's at the bottom level, and shoot if it's at higher levels.
		if(a&&currentLevel==0){
				intake();
		}
		else if(a&&currentLevel>0){
				shoot();
		}
		else if(!a){
				motorGrabL.set(0);
				motorGrabR.set(0);
		}

		//However, this won't work if we're using triggers, or if code is just bad, so here's a fallback.
		if(back&&!hasBall){
				intake();
				intakeActive=true;
		}
		if(back&&hasBall){
				shoot();
				shootActive=true;
		}
		if(!back){
				if(intakeActive){
						motorGrabL.set(0);
						motorGrabR.set(0);
						intakeActive=false;
						hasBall=true;
				}
				if(shootActive){
						motorGrabL.set(0);
						motorGrabR.set(0);
						shootActive=false;
						hasBall=false;
				}
		}
	}
	public void updateLevel(boolean BL, boolean BR, boolean[] PBL, boolean[] PBR){
		//Update bumper - user input for which level to go to.
		if(controls.runOnce(BR,PBR)&&targetLevel<3){
				//if bumper R is pressed and target level is less than 3, increase target level
				targetLevel++;
		}
		else if(controls.runOnce(BL,PBL)&&targetLevel>0){
				//if bumper L is pressed and target level is more than 0, decrease target level
				targetLevel--;
		}

		//Kill if lift hits top or bottom limit switches.
		if(switchBottom.get()){
				liftSpeed=Math.max(0,liftSpeed); //We can't just set it to 0, because the limit switch will continue being held down, disabling the motor for the rest of the game.
				//This ensures the lift speed will be positive.
				currentLevel = 0;
		}
		if(switchTop.get()){
				liftSpeed = Math.min(0,liftSpeed); //See above comments; this ensures lift speed will be negative.
				currentLevel = 4;
		}
		
		//Update limit switches for every level
		if(switchL1.get()){ //If the limit switch for L1 is hit:
				if(currentLevel<1){ //If the current level is less than L1:
						currentLevel++; //Increase current level
				}
				else if(currentLevel>1){ //If the current level is greater than L1:
						currentLevel--; //Decrease current level.
				}
		}
		if(switchL2.get()){ //Same as above.
				if(currentLevel<2){
						currentLevel++;
				}
				else if(currentLevel>2){
						currentLevel--;
				}
		}
		if(switchL3.get()){
				if(currentLevel<3){
						currentLevel++;
				}
				else if(currentLevel>3){
						currentLevel--;
				}
		}

}

public void triggerLift(double TL, double TR) { //This is what we'll use if we can't get limit switches or encoders set up - completely user controlled.
		//From the simulator, it appears that triggerR is [-1,0] and triggerL is [0,1]. Might be different IRL though, so we'll have to test it.
		if(TR<0&&TL==0){
				//Move up if right trigger is pressed and left isn't
				liftSpeed = -TR*speedModifier; //Input is negative, so neg*neg=pos.
		}
		else if(TL>0&&TR==0){
				//Move down if left trigger is pressed and right isn't
				liftSpeed = -TL*speedModifier;
		}
		else if(TL==0&&TR==0){
				liftSpeed=0;
		}
}

public void limitLift(){ //Operate lift based on limit switches
		if(targetLevel<currentLevel){
				liftSpeed=-speedModifier; //go down
		}
		else if(targetLevel>currentLevel){
				liftSpeed=speedModifier; //go up
		}
		else if(targetLevel==currentLevel){
				liftSpeed=0; //stop
		}
}

public void runLift(boolean BL, boolean BR, double TL, double TR, boolean[] PBL, boolean[] PBR){
		updateLevel(BL,BR,PBL,PBR); //limit switches and target level (from bumpers)

		if(liftMode==0){
				triggerLift(TL,TR);
		}
		else if(liftMode==1){
				limitLift();
		}

		motorLift.set(liftSpeed); 
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
		
  public void teleopPeriodic() {

		Scheduler.getInstance().run();

		update();
		dashboardOutput();

		tankDriver(joystickLYAxis,joystickRYAxis);
		climb(buttonX); //On buttonX, bring the arm down and start spinning

		//carriage.runCarriage(controls.buttonA, controls.buttonBack); //Operate carriage (intake/ball shooter). Also calls sucker.succ().
		runLift(bumperL,bumperR,triggerL,triggerR,pairBumperL,pairBumperR); //Operate the lift and grabber. Currently based on triggers; change mode in Lift.java.
	}	
}