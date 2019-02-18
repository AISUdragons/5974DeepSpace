package org.usfirst.frc5974.DeepSpace;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc5974.DeepSpace.Sensors;

public class Controller{

    Sensors sensors = new Sensors();

    //Variables for the Controller
	Joystick controller = new Joystick(0);	//controller
	Joystick keyboard = new Joystick(1); //keyboard (simulate limit switches)
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

    //Simulator doesn't support limit switches so, yeet
    boolean switchBottom = false;
    boolean switchL1 = false;
    boolean switchL2 = false;
    boolean switchL3 = false;
	boolean switchTop = false;
	boolean switchClimber;
	
	boolean[] pairX = {false, false};
	boolean[] pairY = {false, false};
	boolean[] pairA = {false, false};
	boolean[] pairB = {false, false};
	boolean[] pairBumperR = {false,false};
	boolean[] pairBumperL = {false,false};
	boolean[] pairClimber = {false,false};
    
    boolean pressed = false;
	
	public boolean toggle(boolean button, boolean toggle, boolean[] buttonPair) {
		return runOnce(button, buttonPair) ? !toggle : toggle;
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
		//trigger updates DON'T FORGET TO MOVE THIS BEFORE DEADZONE EXECUTION
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

		switchBottom = keyboard.getRawButton(1);
     	switchL1 = keyboard.getRawButton(2);
     	switchL2 = keyboard.getRawButton(3);
     	switchL3 = keyboard.getRawButton(4);
		switchTop = keyboard.getRawButton(5);
		switchClimber = keyboard.getRawButton(6);
    }
    
    public void rumble(double duration){
        controller.setRumble(Joystick.RumbleType.kRightRumble, 0.5);
		controller.setRumble(Joystick.RumbleType.kLeftRumble, 0.5);
		Timer.delay(duration);
		controller.setRumble(Joystick.RumbleType.kRightRumble, 0);
		controller.setRumble(Joystick.RumbleType.kLeftRumble, 0);
    }
}