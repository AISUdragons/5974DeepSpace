package org.usfirst.frc5974.DeepSpace;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DigitalInput;

import org.usfirst.frc5974.DeepSpace.Controller;

public class Sucker{
    Controller controls = new Controller();

    DigitalInput switchSucker = new DigitalInput(5);
    DigitalInput switchBottom = new DigitalInput(6);

    //These will have 2 motor controllers each.
    VictorSP motorsSuckerBase = new VictorSP(7); 
    VictorSP motorsSuckerSpinner = new VictorSP(8);

    double pivotSpeed;
    double suckSpeed=.5;
    double climbSpeed=.75;

    public void succ(){
        motorsSuckerSpinner.set(suckSpeed);
    }

    public void setup(){
        motorsSuckerBase.set(pivotSpeed);
        if(switchSucker.get()){
            motorsSuckerBase.set(0);
        }
    }

    public void climb(){
        if(controls.buttonX){
            if(switchBottom.get()){
                motorsSuckerBase.set(0);
            }
            else{
                motorsSuckerBase.set(pivotSpeed);
            }
            motorsSuckerSpinner.set(climbSpeed);
        }
    }
}