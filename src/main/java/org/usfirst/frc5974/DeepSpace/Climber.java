package org.usfirst.frc5974.DeepSpace;

import edu.wpi.first.wpilibj.VictorSP;

import org.usfirst.frc5974.DeepSpace.Controller;

public class Climber{
    VictorSP climbMotor = new VictorSP(7);
    Controller controls = new Controller();

    public void climb(){
        if(controls.dPad!=0){ //LMAO I'M OUT OF BUTTONS SEND HELP
            climbMotor.set(1);
        }
    }
}