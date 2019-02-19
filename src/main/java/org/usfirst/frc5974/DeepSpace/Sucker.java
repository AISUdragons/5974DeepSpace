package org.usfirst.frc5974.DeepSpace;

public class Sucker extends Robot{

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