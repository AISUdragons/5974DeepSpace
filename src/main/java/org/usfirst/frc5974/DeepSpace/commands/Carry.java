package org.usfirst.frc5974.DeepSpace.commands;

import org.usfirst.frc5974.DeepSpace.Robot;
import org.usfirst.frc5974.DeepSpace.subsystems.*;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Carry extends Command{
    public Carry(){
        requires(Robot.getCarriage());
    }

    
    public void intake(){
        motorGrabL.set(grabSpeed);
        motorGrabR.set(-grabSpeed);
        sucker.succ();
    }

    public void shoot(){
        motorGrabL.set(-grabSpeed);
        motorGrabR.set(grabSpeed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}