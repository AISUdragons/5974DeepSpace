package org.usfirst.frc5974.DeepSpace;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Lift
{
    VictorSP motorLift = new VictorSP(4);

    private double distanceModifier = 1;
    private double speedModifier = 1;

    double targetLevel = 0;
    boolean kill = false;
    double currentLevel = 0;
    int baseLevel = 0;
    double previousPosition =0;
    double Position = motorLift.getPosition();

public void updateLift() 
    {
        //Only runs if kill is false
        if (!kill)
          {
            //Makes sure that prevRPpos is one update behind Position
            previousPosition = Position;
            Position = motorLift.getPosition();
            
            //Manages baseLevel
            if (motorLift.getSpeed() > 0 && Position < previousPosition)
            {
                baseLevel ++;
            }
            else if (motorLift.getSpeed() < 0 && Position > previousPosition)
            {
                baseLevel --;
            }

            //Sets currentLevel
            currentLevel = baseLevel + Position;

            //Moves the motor if currentLevel is not euqual to targetLevel
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
    }
}