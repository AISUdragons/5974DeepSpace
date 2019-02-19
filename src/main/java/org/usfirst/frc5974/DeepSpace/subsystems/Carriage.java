package org.usfirst.frc5974.DeepSpace.subsystems;

import org.usfirst.frc5974.DeepSpace.Robot;

public class Carriage extends Robot{
    //Lift lift = new Lift();
    //Sucker sucker = new Sucker();
    
    double grabSpeed = 1; //grabber/intake motor speed
    boolean hasBall = false;
    boolean intakeActive=false;
    boolean shootActive=false;

    public void runCarriage(boolean a, boolean back){
        //Theoretically, this will take in balls if it's at the bottom level, and shoot if it's at higher levels.
        if(a&&lift.currentLevel==0){
            intake();
        }
        else if(a&&lift.currentLevel>0){
            shoot();
        }
        else if(!a){
            motorGrabL.set(0);
            motorGrabR.set(0);
        }

        //However, this won't work if we're using triggers, or if code is just bad, so here's a fallback.
        if(back&&!hasBall){
            intake();
            intakeActive=true;
        }
        if(back&&hasBall){
            shoot();
            shootActive=true;
        }
        if(!back){
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