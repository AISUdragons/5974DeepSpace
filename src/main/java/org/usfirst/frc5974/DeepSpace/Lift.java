package org.usfirst.frc5974.DeepSpace;

import org.usfirst.frc5974.DeepSpace.Controller;
import org.usfirst.frc5974.DeepSpace.Sucker;

public class Lift extends Robot{
    Controller controls = new Controller();
    Sucker sucker = new Sucker();

    //Variables
    double speedModifier = .5; //change this to make lift move faster or slower
    int targetLevel = 0; //level it's supposed to go to
    double currentLevel = 0; //level it's currently at
    double liftSpeed = 0; //the value we set motorLift to
    int liftMode = 0; //0 is trigger, 1 is limit switch.

    public void updateLevel(boolean BL, boolean BR){
        //Update bumper - user input for which level to go to.
        if(controls.runOnce(BR,controls.pairBumperR)&&targetLevel<3){
            //if bumper R is pressed and target level is less than 3, increase target level
            targetLevel++;
        }
        else if(controls.runOnce(BL,controls.pairBumperL)&&targetLevel>0){
            //if bumper L is pressed and target level is more than 0, decrease target level
            targetLevel--;
        }

        //Kill if lift hits top or bottom limit switches.
        if(switchBottom.get()){
            liftSpeed=Math.max(0,liftSpeed); //We can't just set it to 0, because the limit switch will continue being held down, disabling the motor for the rest of the game.
            //This ensures the lift speed will be positive.
            currentLevel = 0;
        }
        if(switchTop.get()){
            liftSpeed = Math.min(0,liftSpeed); //See above comments; this ensures lift speed will be negative.
            currentLevel = 4;
        }
        
        //Update limit switches for every level
        if(switchL1.get()){ //If the limit switch for L1 is hit:
            if(currentLevel<1){ //If the current level is less than L1:
                currentLevel++; //Increase current level
            }
            else if(currentLevel>1){ //If the current level is greater than L1:
                currentLevel--; //Decrease current level.
            }
        }
        if(switchL2.get()){ //Same as above.
            if(currentLevel<2){
                currentLevel++;
            }
            else if(currentLevel>2){
                currentLevel--;
            }
        }
        if(switchL3.get()){
            if(currentLevel<3){
                currentLevel++;
            }
            else if(currentLevel>3){
                currentLevel--;
            }
        }

    }

    public void triggerLift(double TL, double TR) { //This is what we'll use if we can't get limit switches or encoders set up - completely user controlled.
        //From the simulator, it appears that triggerR is [-1,0] and triggerL is [0,1]. Might be different IRL though, so we'll have to test it.
        if(TR<0&&TL==0){
            //Move up if right trigger is pressed and left isn't
            liftSpeed = -TR*speedModifier; //Input is negative, so neg*neg=pos.
        }
        else if(TL>0&&TR==0){
            //Move down if left trigger is pressed and right isn't
            liftSpeed = -TL*speedModifier;
        }
        else if(TL==0&&TR==0){
            liftSpeed=0;
        }
    }

    public void limitLift(){ //Operate lift based on limit switches
        if(targetLevel<currentLevel){
            liftSpeed=-speedModifier; //go down
        }
        else if(targetLevel>currentLevel){
            liftSpeed=speedModifier; //go up
        }
        else if(targetLevel==currentLevel){
            liftSpeed=0; //stop
        }
    }

    public void runLift(boolean BL, boolean BR, double TL, double TR){
        updateLevel(BL,BR); //limit switches and target level (from bumpers)

        if(liftMode==0){
            triggerLift(TL,TR);
        }
        else if(liftMode==1){
            limitLift();
        }

        motorLift.set(liftSpeed); 
    }
}