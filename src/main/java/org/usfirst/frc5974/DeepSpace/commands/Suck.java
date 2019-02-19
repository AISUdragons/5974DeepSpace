package org.usfirst.frc5974.DeepSpace.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DigitalInput;

import org.usfirst.frc5974.DeepSpace.Robot;
import org.usfirst.frc5974.DeepSpace.subsystems.*;

public class Suck extends Command{

    public Sucker(){
        requires(Robot.sucker);
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

    @Override
    protected boolean isFinished() {
        return true;
    }
}