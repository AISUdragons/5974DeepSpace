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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc5974.DeepSpace.commands.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.vision.VisionThread;
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

//Motors
	//Carriage motors
		VictorSP motorGrabL = new VictorSP(4);
		VictorSP motorGrabR = new VictorSP(5);
	//Sucker motors
		//These will have 2 motor controllers each.
		VictorSP motorSuckerBaseL = new VictorSP(7);
		VictorSP motorSuckerBaseR = new VictorSP(6);
		SpeedControllerGroup motorsSuckerBase = new SpeedControllerGroup(motorSuckerBaseL,motorSuckerBaseR);
		VictorSP motorSuckerSpinL = new VictorSP(2); 
		VictorSP motorSuckerSpinR = new VictorSP(3);
		SpeedControllerGroup motorsSuckerSpinner = new SpeedControllerGroup(motorSuckerSpinL,motorSuckerSpinR);
	//Lift motor
		VictorSP motorLift = new VictorSP(8);
	//Drive motors
		//These have 2 motor controllers each
		VictorSP motorsRight = new VictorSP(0);
		VictorSP motorsLeft = new VictorSP(1); 

//Limit switches (emulated)
	Joystick keyboard = new Joystick(1); //Not sure if this will work on actual driverstation. We'll see.
	//Sucker limit switches
		boolean  switchSucker;
		boolean  switchBase;
	//Lift limit switches
		boolean  switchBottom;
		boolean  switchL1;
		boolean  switchL2;
		boolean  switchL3;
		boolean  switchTop;
		boolean switchCarriage;
		boolean keyboardEmulation = true;
		
//Variables
	//Camera variables
		private static final int IMG_WIDTH = 240;
		private static final int IMG_HEIGHT = 180;
		private static final int fps = 20;
		private VisionThread visionThread;
		double centerX = 0.0;
		private final Object imgLock = new Object();
	//Sucker variables
	double pivotSpeed=.5;
    double suckSpeed=.5;
    double climbSpeed=.75;

  //Lift variables
    double speedModifier = .5; //change this to make lift move faster or slower
    int targetLevel = 0; //level it's supposed to go to
    double currentLevel = 0; //level it's currently at
    double liftSpeed = 0; //the value we set motorLift to
	int liftMode = 1; //0 is trigger, 1 is limit switch.

  //Carriage variables
  int carriageMode=2; //0:levels (A), 1: toggle (back), 2: limit switch (back)
  double grabSpeed = 1; //grabber/intake motor speed
    boolean hasBall = false;
    boolean intakeActive=false;
	boolean shootActive=false;
	boolean shot=false;
	boolean grabbed=false;

  //Drive Variables
		boolean fastBool = false;		//speed mode: true = fast mode, false = slow mode
		boolean driveNormal = true; 	//drive mode: true = normal tank drive, false = drive straight
		double slowModifier = 0.75; //Set slow mode speed
	//Controller Variables
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
		boolean[] pairX = {false, false};
		boolean[] pairY = {false, false};
		boolean[] pairA = {false, false};
		boolean[] pairB = {false, false};
		boolean[] pairBumperR = {false,false};
		boolean[] pairBumperL = {false,false};
		boolean pressed = false;
	
	//Misc variables
		Timer timer = new Timer();
		//For tracking iterative updates
		int track = 0;
		int check = 10;
		//Sendable chooser on SmartDashboard - we can use this to choose different autonomous options, etc from smartdash
		Command autonomousCommand;
		SendableChooser<Command> chooser = new SendableChooser<>();
		public static OI oi;
	
	DifferentialDrive driver = new DifferentialDrive(motorsLeft, motorsRight);

//Controller
	Joystick controller = new Joystick(0);
	public boolean toggle(boolean button, boolean toggle, boolean[] buttonPair) {
		return runOnce(button, buttonPair) ? !toggle : toggle;
	}
	public boolean checkButton(boolean pressed, boolean toggle, int portNum) {
		//When the button is pushed, once it is released, its toggle is changed
		if (pressed) {
			toggle = !toggle;
			while (pressed) {		//if the riolog complains about loop being overrun or whatever, this code is probably causing it
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

		//Limit switch emulation
		if(keyboardEmulation){
			switchSucker = keyboard.getRawButton(1);
			switchBase = keyboard.getRawButton(2);
			switchBottom = keyboard.getRawButton(3);
			switchL1 = keyboard.getRawButton(4);
			switchL2 = keyboard.getRawButton(5);
			switchL3 = keyboard.getRawButton(6);
			switchTop = keyboard.getRawButton(7);
			switchCarriage = keyboard.getRawButton(8);
		}
		else{
			switchSucker = true;
			switchBase = false;
		}
	}		
	public void rumble(double duration){
		controller.setRumble(Joystick.RumbleType.kRightRumble, 0.5);
		controller.setRumble(Joystick.RumbleType.kLeftRumble, 0.5);
		Timer.delay(duration);
		controller.setRumble(Joystick.RumbleType.kRightRumble, 0);
		controller.setRumble(Joystick.RumbleType.kLeftRumble, 0);
	}


//Driver
	public void tankDriver(double L, double R) {	//left joystick controls left wheels, right joystick controls right wheels
		if (fastBool) {
			driver.tankDrive(-L, -R);
		} 
		else {
			driver.tankDrive(-slowModifier*L, -slowModifier*R);
		}
	}

//Sucker
	public void intake(boolean enable){
		if(enable){
			motorGrabL.set(grabSpeed);
			motorGrabR.set(-grabSpeed);
		}
		else{
			motorGrabL.set(0);
			motorGrabR.set(0);
		}
	}
	public void suckerSetup(){
		motorsSuckerBase.set(pivotSpeed);
		double moveSpeed = pivotSpeed;
		if(switchSucker){
			//System.out.println("switchSucker");
			while(motorsSuckerBase.get()!=0){
				if(moveSpeed>0){
					moveSpeed=moveSpeed-.0001;
				}
				motorsSuckerBase.set(moveSpeed); //gradually slow when hit limit switch. Simulator says this'll take under a second; may have to finetune.
				if(moveSpeed<.001){
					motorsSuckerBase.set(0);
					break;
				}
			}
		}
	}
	public void succ(boolean enable){
		if(enable){
			motorsSuckerSpinner.set(suckSpeed);
		}
		else if(!buttonX&&!buttonY){
			motorsSuckerSpinner.set(0);
		}
	}
	public void climb(boolean X, boolean Y){
		double climbSetSpeed=0;
		if(X){
			climbSetSpeed=pivotSpeed;
		}
		else if(Y){
			climbSetSpeed=-pivotSpeed;
		}
		if(switchBase){
			climbSetSpeed=Math.max(0,climbSetSpeed);
		}
		if(X||Y){
			motorsSuckerSpinner.set(climbSpeed);
		}
		else{
			climbSetSpeed=0;
		}
		motorsSuckerBase.set(climbSetSpeed);
	}	


//Carriage
	public void shoot(){
		motorGrabL.set(-grabSpeed);
		motorGrabR.set(grabSpeed);
	}
	public void runCarriage(boolean a, boolean back){
		//Theoretically, this will take in balls if it's at the bottom level, and shoot if it's at higher levels.
		if(carriageMode==0){
			if(a&&currentLevel==0){
				intake(true);
				succ(true);
			}
			else if(a&&currentLevel>0){
				shoot();
			}
			else if(!a){
				intake(false);
				succ(false);
		}}
		//However, this won't work if we're using triggers, or if code is just bad, so here's a fallback.
		if(carriageMode==1){
			if(back&&!hasBall){
				intake(true);
				succ(true);
				intakeActive=true;
			}
			if(back&&hasBall){
				shoot();
				shootActive=true;
			}
			if(!back){
				if(intakeActive){
					intake(false);
					succ(false);
					intakeActive=false;
					hasBall=true;
				}
				if(shootActive){
					intake(false);
					succ(false);
					shootActive=false;
					hasBall=false;
				}
			}	}
		//Limit switch
		if(carriageMode==2){
			if(switchCarriage){
				grabbed=true;
				System.out.println("hasBall");
			}
			if(back&&hasBall){
				shoot();
				shot=true;
				grabbed=false;
			}
			if(back&&!hasBall){
				intake(true);
				succ(true);
			}
			if(!back){
				intake(false);
				succ(false);
				if(shot){
					hasBall=false;
				}
				if(grabbed){
					hasBall=true;
				}
			}
		}
	}

//Lift
	public void updateLevel(boolean BL, boolean BR, boolean[] PBL, boolean[] PBR){
		//Update bumper - user input for which level to go to.
		if(runOnce(BR,PBR)&&targetLevel<3){
			//if bumper R is pressed and target level is less than 3, increase target level
			targetLevel++;
		}
		else if(runOnce(BL,PBL)&&targetLevel>0){
			//if bumper L is pressed and target level is more than 0, decrease target level
			targetLevel--;
		}
		//Kill if lift hits top or bottom limit switches.
		if(keyboardEmulation){
			if(switchBottom){
				liftSpeed=Math.max(0,liftSpeed); //We can't just set it to 0, because the limit switch will continue being held down, disabling the motor for the rest of the game.
				//This ensures the lift speed will be positive.
				currentLevel = 0;
			}
			if(switchTop){
				liftSpeed = Math.min(0,liftSpeed); //See above comments; this ensures lift speed will be negative.
				currentLevel = 4;
			}
			//Update limit switches for every level
			if(switchL1){ //If the limit switch for L1 is hit:
				if(currentLevel<1){ //If the current level is less than L1:
						currentLevel++; //Increase current level
				}
				else if(currentLevel>1){ //If the current level is greater than L1:
					currentLevel--; //Decrease current level.				
				}
			}
			if(switchL2){ //Same as above.
				if(currentLevel<2){
						currentLevel++;
				}
				else if(currentLevel>2){
					currentLevel--;
				}
			}
			if(switchL3){
				if(currentLevel<3){
					currentLevel++;
				}
				else if(currentLevel>3){
					currentLevel--;
				}
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

//Camera
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

//Internal commands
	public void update() {					//updates everything
		updateController();
		//toggle checks
		fastBool = toggle(buttonB, fastBool, pairB);	//toggles boolean if button is pressed
	}
	public void dashboardOutput() {			//sends and displays data to smart dashboard
		SmartDashboard.putBoolean("Fast Mode", fastBool);
		SmartDashboard.putNumber("Lift Target Level",targetLevel);
		SmartDashboard.putNumber("Lift Current Level",currentLevel);
		
		SmartDashboard.getBoolean("Sucker at Angle",switchSucker);
		SmartDashboard.getBoolean("Sucker at Bottom",switchBase);
		SmartDashboard.getBoolean("Lift at Bottom",switchBottom);
		SmartDashboard.getBoolean("Lift Level 1",switchL1);
		SmartDashboard.getBoolean("Lift Level 2",switchL2);
		SmartDashboard.getBoolean("Lift Level 3",switchL3);
		SmartDashboard.getBoolean("Lift at Top",switchTop);
		SmartDashboard.getBoolean("Carriage/ball detector",switchCarriage);
	}

//Robot blocks
  @Override
  public void robotInit() {
    oi = new OI();
		//I think this is how we choose different autonomous code options from smartdash
    chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());
		SmartDashboard.putData("Auto mode", chooser);
		driver.setRightSideInverted(true); //TODO: invert right side or nah?
		timer.start();
		//cameraInit(); //Initialize camera/image processing
  }
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
		if (autonomousCommand != null) autonomousCommand.start();
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
		rumble(0.5);
		suckerSetup(); //Set sucker to limit switch to prepare for gameplay
	}
	@Override	
  public void teleopPeriodic() {
		Scheduler.getInstance().run();
		update();
		dashboardOutput();
		tankDriver(joystickLYAxis,joystickRYAxis);
		climb(buttonX,buttonY); //X climb, Y retract
		runCarriage(buttonA, buttonBack); //Operate carriage (intake/ball shooter). Also calls sucker.succ().
		runLift(bumperL,bumperR,triggerL,triggerR,pairBumperL,pairBumperR); //Operate the lift and grabber. Currently based on triggers; change mode in Lift.java.
		if(buttonY){
			System.out.println("hasBall:"+hasBall);
		}
	}	

}