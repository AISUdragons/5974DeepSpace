package org.usfirst.frc5974.DeepSpace;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Encoder;

import org.usfirst.frc5974.DeepSpace.Controller;

public class Lift
{
    VictorSP motorLift = new VictorSP(4);
    Controller controls = new Controller();
    int channelA = 0;
    int channelB = 1;
    Encoder encoder = new Encoder(channelA,channelB);
    private double distanceModifier = 1;
    private double speedModifier = .5;

    int targetLevel = 0;
    boolean kill = false;
    double currentLevel = 0;
    int baseLevel = 0;
    double position = 0;
    double distanceTraveled = 0;
    final int[] heights = {0,1,2,3}; //heights for each goal TODO: update goal heights to actual

    //double Position = motorLift.getPosition();
    //We're using a geared CIM, not a servo, so this probably won't work. I'll try to figure out how to use encoders.
    //i can't find documentation for the encoder we're using?? so let's just map "up" to the right trigger and "down" to the left, for now at least.

public void triggerLift() { //This is what we'll use if we can't get encoders working.
        if(!kill){
            if(controls.triggerR>0&&controls.triggerL==0){
                //Move up if right trigger is pressed and left isn't
                motorLift.set(controls.triggerL*speedModifier);
            }
            else if(controls.triggerL>0&&controls.triggerR==0){
                //Move down if left trigger is pressed and right isn't
                motorLift.set(-controls.triggerL*speedModifier);
            }
        }
        else{
            motorLift.set(0);
        }
    }
public void encoderLift(){
    if(!kill){
        /*
        prompt for level 0, 1, 2, or 3
        calculate distance lift has traveled
        if it hasn't gone enough, keep motor on
        if it has, turn motor off
        */
        //prompt L0-3
        if(controls.bumperR&&targetLevel<3){
            //if bumper R is pressed and target level is less than 4, increase target level
            targetLevel++;
        }
        else if(controls.bumperL&&targetLevel>0){
            //if bumper L is pressed and target level is more than 0, decrease target level
            targetLevel--;
        }

        //calculate distance lift has traveled, move accordingly
        //let's be honest, this is not going to work. i wish we could actually test stuff :(
        if(encoder.getDistance()<heights[targetLevel]){
            motorLift.set(speedModifier);
        }
        else if(encoder.getDistance()>heights[targetLevel]){
            motorLift.set(-speedModifier);
        }
        else if(encoder.getDistance()==heights[targetLevel]){ //it won't be exact, so you'll probably have to change this to a distance-1<target height<distance+1 type of thing
            motorLift.set(0);
        }
    }
    else{
        motorLift.set(0);
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
}
}