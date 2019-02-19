package org.usfirst.frc5974.DeepSpace;

import edu.wpi.first.wpilibj.VictorSP;

import org.usfirst.frc5974.DeepSpace.Controller;
import org.usfirst.frc5974.DeepSpace.Lift;
import org.usfirst.frc5974.DeepSpace.Sucker;

public class Carriage{
    Controller controls = new Controller();
    Lift lift = new Lift();
    Sucker sucker = new Sucker();

    VictorSP motorGrabL = new VictorSP(5);
    VictorSP motorGrabR = new VictorSP(6);
    
    double grabSpeed = 1; //grabber/intake motor speed
    boolean hasBall = false;
    boolean intakeActive=false;
    boolean shootActive=false;

    public void runCarriage(){
        //Theoretically, this will take in balls if it's at the bottom level, and shoot if it's at higher levels.
        if(controls.buttonA&&lift.currentLevel==0){
            intake();
        }
        else if(controls.buttonA&&lift.currentLevel>0){
            shoot();
        }
        else if(!controls.buttonA){
            motorGrabL.set(0);
            motorGrabR.set(0);
        }

        //However, this won't work if we're using triggers, or if code is just bad, so here's a fallback.
        if(controls.buttonBack&&!hasBall){
            intake();
            intakeActive=true;
        }
        if(controls.buttonBack&&hasBall){
            shoot();
            shootActive=true;
        }
        if(!controls.buttonBack){
            if(intakeActive){
                motorGrabL.set(0);
                motorGrabR.set(0);
                intakeActive=false;
                hasBall=true;
            }
            if(shootActive){
                motorGrabL.set(0);
                motorGrabR.set(0);
                shootActive=false;
                hasBall=false;
            }
        }
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
}