package org.usfirst.frc5974.DeepSpace;

import edu.wpi.first.wpilibj.VictorSP;

import org.usfirst.frc5974.DeepSpace.Controller;

public class Climber{
    VictorSP motorClimb = new VictorSP(7);
    Controller controls = new Controller();
    double climbSpeed = 1;

    public void climb(){
        if(controls.buttonStart){
            motorClimb.set(climbSpeed);
        }
        else if(!controls.buttonStart){
            motorClimb.set(0);
        }
    }
}