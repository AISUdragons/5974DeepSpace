package org.usfirst.frc5974.DeepSpace;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput; //limit switches

public class Lift{
    VictorSP motorLift = new VictorSP(4);
    Controller controls = new Controller();
    
    //Encoder constructor
    int channelA = 0;
    int channelB = 1;
    Encoder encoder = new Encoder(channelA,channelB);

    //Limit switch constructors
    DigitalInput switchBottom = new DigitalInput(0); //TODO: Set limit switches to the correct ports
    DigitalInput switchL1 = new DigitalInput(1);
    DigitalInput switchL2 = new DigitalInput(2);
    DigitalInput switchL3 = new DigitalInput(3);
    DigitalInput switchTop = new DigitalInput(4);

    //Variables
    double speedModifier = .5; //change this to make lift move faster or slower
    int targetLevel = 0; //level it's supposed to go to
    double currentLevel = 0; //level it's currently at
    int[] heights = {0,1,2,3}; //heights for each goal (only used in encoder mode)
    double liftSpeed = 0; //the value we set motorLift to
    int liftMode = 0; //0 is trigger, 1 is switch, 2 is encoder. We can probably remove the extra code once we know which one we're using.

    public void updateLevel(){
        //Update bumper - user input for which level to go to.
        if(controls.bumperR&&targetLevel<3){
            //if bumper R is pressed and target level is less than 3, increase target level
            targetLevel++;
        }
        else if(controls.bumperL&&targetLevel>0){
            //if bumper L is pressed and target level is more than 0, decrease target level
            targetLevel--;
        }

        //Kill if lift hits top or bottom limit switches.
        if(switchBottom.get()){
            liftSpeed=Math.max(0,liftSpeed); //We can't just set it to 0, because the limit switch will continue being held down, disabling the motor for the rest of the game.
            //This ensures the lift speed will be positive.
        }
        if(switchTop.get()){
            liftSpeed = Math.min(0,liftSpeed); //See above comments; this ensures lift speed will be negative.
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

    public void triggerLift() { //This is what we'll use if we can't get limit switches or encoders set up - completely user controlled.
        if(controls.triggerR>0&&controls.triggerL==0){
            //Move up if right trigger is pressed and left isn't
            liftSpeed = controls.triggerL*speedModifier;
        }
        else if(controls.triggerL>0&&controls.triggerR==0){
            //Move down if left trigger is pressed and right isn't
            liftSpeed = -controls.triggerL*speedModifier;
        }
        else if(controls.triggerL==0&&controls.triggerR==0){
            liftSpeed=0;
        }
    }

    public void limitLift(){ //Operate lift based on limit switches
        if(targetLevel<currentLevel){
            liftSpeed=speedModifier; //go up
        }
        else if(targetLevel>currentLevel){
            liftSpeed=-speedModifier; //go down
        }
        else if(targetLevel==currentLevel){
            liftSpeed=0; //stop
        }
    }

    public void encoderLift(){ //Operate lift based on encoder (very unlikely this will happen)
        if(encoder.getDistance()<heights[targetLevel]){
            liftSpeed=speedModifier;
        }
        else if(encoder.getDistance()>heights[targetLevel]){
            liftSpeed=-speedModifier;
        }
        else if(encoder.getDistance()<heights[targetLevel]+.5&&encoder.getDistance()>heights[targetLevel]-.5){
            liftSpeed=0;
        }
    }

    public void runLift(){
        updateLevel(); //limit switches and target level (from bumpers)

        if(liftMode==0){
            triggerLift();
        }
        else if(liftMode==1){
            limitLift();
        }
        else if(liftMode==2){
            encoderLift();
        }

        motorLift.set(liftSpeed); 

    }
}

//I deleted basically everything that was here previously. If you want it back, check Github :P