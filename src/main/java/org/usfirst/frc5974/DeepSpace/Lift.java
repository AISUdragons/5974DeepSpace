package org.usfirst.frc5974.DeepSpace;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput; //limit switches

import org.usfirst.frc5974.DeepSpace.Controller;

public class Lift
{
    int channelA = 0;
    int channelB = 1;
    Encoder encoder = new Encoder(channelA,channelB);

    DigitalInput switchBottom = new DigitalInput(0); //TODO: Set limit switches to the correct ports
    DigitalInput switchL1 = new DigitalInput(1);
    DigitalInput switchL2 = new DigitalInput(2);
    DigitalInput switchL3 = new DigitalInput(3);
    DigitalInput switchTop = new DigitalInput(4);
    
    VictorSP motorLift = new VictorSP(4);
    Controller controls = new Controller();
    private double distanceModifier = 1;
    private double speedModifier = .5;

    int targetLevel = 0;
    boolean kill = false;
    double currentLevel = 0;
    int baseLevel = 0;
    double position = 0;
    double distanceTraveled = 0;
    final int[] heights = {0,1,2,3}; //heights for each goal TODO: update goal heights to actual (only if using encoder)
    double liftSpeed = 0;

    int liftMode = 0; //0 is trigger, 1 is switch, 2 is encoder. We can probably remove the extra code once we know which one we're using

    public void updateLevel(){
        //Update bumper - user input for which level to go to
        if(controls.bumperR&&targetLevel<3){
            //if bumper R is pressed and target level is less than 4, increase target level
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
        if(switchL1.get()){
            if(currentLevel<1){
                currentLevel++;
            }
            else{
                currentLevel--;
            }
        }
        if(switchL2.get()){
            if(currentLevel<2){
                currentLevel++;
            }
            else{
                currentLevel--;
            }
        }
        if(switchL3.get()){
            if(currentLevel<3){
                currentLevel++;
            }
            else{
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
            liftSpeed=speedModifier;
        }
        else if(targetLevel>currentLevel){
            liftSpeed=speedModifier;
        }
        else if(targetLevel==currentLevel){
            liftSpeed=0;
        }
    }

    public void encoderLift(){ //Operate lift based on encoder (very unlikely this will happen)
        if(encoder.getDistance()<heights[targetLevel]){
            liftSpeed=speedModifier;
        }
        else if(encoder.getDistance()>heights[targetLevel]){
            liftSpeed=-speedModifier;
        }
        else if(encoder.getDistance()==heights[targetLevel]){ //it won't be exact, so you'll probably have to change this to a distance-1<target height<distance+1 type of thing. 
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

        /*
        Servo-based code (won't work)
        //Only runs if kill is false
        if (!kill){
            //Makes sure that prevRPpos is one update behind Position
            previousPosition = Position;
            Position = motorLift.getPosition();
            
            //Manages baseLevel
            if (motorLift.getSpeed() > 0 && Position < previousPosition){
                baseLevel ++;
            }
            else if (motorLift.getSpeed() < 0 && Position > previousPosition)
            {
                baseLevel --; 
            }

            //Sets currentLevel
            currentLevel = baseLevel + Position;

            //Moves the motor if currentLevel is not equal to targetLevel
            if (currentLevel > targetLevel+0.1)
            {
                motorLift.setSpeed(1*speedModifier);
            }
            else if (currentLevel < targetLevel-0.1)
            {
                motorLift.setSpeed(-1*speedModifier);
            }
            else
            {
                motorLift.setSpeed(0);
            }
        }
        else
        {
            motorLift.setSpeed(0);
        }
        */