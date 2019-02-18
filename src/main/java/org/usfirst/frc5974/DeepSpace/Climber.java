package org.usfirst.frc5974.DeepSpace;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import org.usfirst.frc5974.DeepSpace.Controller;

public class Climber{
    VictorSP motorClimbL = new VictorSP(7);
    VictorSP motorClimbR = new VictorSP(8);
    SpeedControllerGroup motorsClimb = new SpeedControllerGroup(motorClimbL,motorClimbR);
    Controller controls = new Controller();
    double climbSpeed = 1;

    public void climb(){
        if(controls.buttonStart){
            motorsClimb.set(climbSpeed);
        }
        else if(!controls.buttonStart){
            motorsClimb.set(0);
        }
    }
}