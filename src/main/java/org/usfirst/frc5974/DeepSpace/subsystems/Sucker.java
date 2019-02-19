package org.usfirst.frc5974.DeepSpace.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DigitalInput;

public class Sucker extends Subsystem{

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

    public void initDefaultCommand(){
    }

    public void succ(){
        motorsSuckerSpinner.set(suckSpeed);
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
}