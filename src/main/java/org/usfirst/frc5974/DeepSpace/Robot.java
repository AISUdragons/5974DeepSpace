// 
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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc5974.DeepSpace.commands.*;
import org.usfirst.frc5974.DeepSpace.subsystems.*;

//extra imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    public static OI oi;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    Joystick riderController = new Joystick(1);			//controller
	Joystick masterController = new Joystick(0);
	
	VictorSP motorRB = new VictorSP(0); //motor right back //THIS IS INVERTED; USE NEGATIVES TO GO FORWARDS
	VictorSP motorRF = new VictorSP(1); //motor right front //THIS IS INVERTED; USE NEGATIVES TO GO FORWARDS
	VictorSP motorLB = new VictorSP(3); //motor left back 
	VictorSP motorLF = new VictorSP(2); //motor left front 
	
	double masterJoystickLXAxis;			//left joystick x-axis
	double masterJoystickLYAxis;			//left joystick y-axis
	double masterJoystickRXAxis;			//right joystick x-axis
	double masterJoystickRYAxis;			//right joystick y-axis
	double masterTriggerL;				//left trigger
	double masterTriggerR;				//right trigger
	
	double riderJoystickLXAxis;			//left joystick x-axis
	double riderJoystickLYAxis;			//left joystick y-axis
	double riderJoystickRXAxis;			//right joystick x-axis
	double riderJoystickRYAxis;			//right joystick y-axis
	double riderTriggerL;				//left trigger
	double riderTriggerR;				//right trigger
	
	
	
	public void joystickDeadZone() {		//sets dead zone for joysticks		//TODO test this
		if (masterJoystickLXAxis <= 0.075 && masterJoystickLXAxis >= -0.075) {
			masterJoystickLXAxis = 0;
		} if (masterJoystickLYAxis <= 0.075 && masterJoystickLYAxis >= -0.075) {
			masterJoystickLYAxis = 0;
		}
		if (masterJoystickRXAxis <= 0.075 && masterJoystickRXAxis >= -0.075) {
			masterJoystickRXAxis = 0;
		} if (masterJoystickRYAxis <= 0.075 && masterJoystickRYAxis >= -0.075) {
			masterJoystickRYAxis = 0;
		}
		
		if (riderJoystickLXAxis <= 0.075 && riderJoystickLXAxis >= -0.075) {
			riderJoystickLXAxis = 0;
		} if (riderJoystickLYAxis <= 0.075 && riderJoystickLYAxis >= -0.075) {
			riderJoystickLYAxis = 0;
		}
		if (riderJoystickRXAxis <= 0.075 && riderJoystickRXAxis >= -0.075) {
			riderJoystickRXAxis = 0;
		} if (riderJoystickRYAxis <= 0.075 && riderJoystickRYAxis >= -0.075) {
			riderJoystickRYAxis = 0;
		}
	}
	
	public void updateController() {
		masterJoystickLXAxis = masterController.getRawAxis(0);		//returns a value [-1,1]
		masterJoystickLYAxis = masterController.getRawAxis(1);		//returns a value [-1,1]
		
		masterJoystickRXAxis = masterController.getRawAxis(4);		//returns a value [-1,1]
		masterJoystickRYAxis = masterController.getRawAxis(5);		//returns a value [-1,1]
		
		masterTriggerL = masterController.getRawAxis(2);		//returns a value [0,1]
		masterTriggerR = masterController.getRawAxis(3);		//returns a value [0,1]
		
		
		
		riderJoystickLXAxis = riderController.getRawAxis(0);		//returns a value [-1,1]
		riderJoystickLYAxis = riderController.getRawAxis(1);		//returns a value [-1,1]
		
		riderJoystickRXAxis = riderController.getRawAxis(4);		//returns a value [-1,1]
		riderJoystickRYAxis = riderController.getRawAxis(5);		//returns a value [-1,1]
		
		riderTriggerL = riderController.getRawAxis(2);		//returns a value [0,1]
        riderTriggerR = riderController.getRawAxis(3);		//returns a value [0,1]
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {

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
		masterController.setRumble(Joystick.RumbleType.kRightRumble, 0.5);
		masterController.setRumble(Joystick.RumbleType.kLeftRumble, 0.5);
		Timer.delay(0.5);
		masterController.setRumble(Joystick.RumbleType.kRightRumble, 0);
		masterController.setRumble(Joystick.RumbleType.kLeftRumble, 0);
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

        updateController();
		
		if (masterTriggerL > 0 || masterTriggerR > 0) {
			motorRB.set(riderJoystickRYAxis/2);
			motorRF.set(riderJoystickRYAxis/2);
			motorLB.set(-riderJoystickLYAxis/2);
			motorLF.set(-riderJoystickLYAxis/2);
		}
		else {
			motorRB.set(masterJoystickRYAxis/2);
			motorRF.set(masterJoystickRYAxis/2);
			motorLB.set(-masterJoystickLYAxis/2);
			motorLF.set(-masterJoystickLYAxis/2);
		}
    }
}
