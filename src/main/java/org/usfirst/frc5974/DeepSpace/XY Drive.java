package org.usfirst.frc5974.DeepSpace;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc5974.DeepSpace.commands.*;

import edu.wpi.first.wpilibj.*;


import edu.wpi.first.cameraserver.CameraServer;


public class Robot extends TimedRobot {

    SendableChooser<Command> chooser = new SendableChooser<>();

	VictorSP motorRB = new VictorSP(1);
	VictorSP motorRF = new VictorSP(0);  
	VictorSP motorLB = new VictorSP(3);
	VictorSP motorLF = new VictorSP(2); 

	Joystick controller = new Joystick(0);

	
	Timer timer = new Timer();
	Timer timerTest = new Timer();

	private Accelerometer accel; 
	double xVal;
	double yVal;
	double zVal;
	private Gyro gyro;
	double angle;

    public static OI oi;

    @Override
	public void robotInit() 
	{

    }

    @Override
	public void disabledInit()
	{
    }

    @Override
	public void disabledPeriodic()
	{
    }

    @Override
	public void autonomousInit() 
	{
    }

    @Override
	public void autonomousPeriodic() 
	{
    }

    @Override
	public void teleopInit() 
	{
	}

    @Override
	public void teleopPeriodic() 
	{
    }
}