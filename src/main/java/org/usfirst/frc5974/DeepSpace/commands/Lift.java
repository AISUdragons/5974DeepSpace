package org.usfirst.frc5974.DeepSpace.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;

import org.usfirst.frc5974.DeepSpace.Robot;
import org.usfirst.frc5974.DeepSpace.subsystems.*;

public class Lift extends Command{
    public Lift(){
        requires(Robot.getLift());
    }

    public void updateLevel(boolean BL, boolean BR, boolean[] PBL, boolean[] PBR){
        //Update bumper - user input for which level to go to.
        if(controls.runOnce(BR,PBR)&&targetLevel<3){
            //if bumper R is pressed and target level is less than 3, increase target level
            targetLevel++;
        }
        else if(controls.runOnce(BL,PBL)&&targetLevel>0){
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

    public void runLift(boolean BL, boolean BR, double TL, double TR, boolean[] PBL, boolean[] PBR){
        updateLevel(BL,BR,PBL,PBR); //limit switches and target level (from bumpers)

        if(liftMode==0){
            triggerLift(TL,TR);
        }
        else if(liftMode==1){
            limitLift();
        }

        motorLift.set(liftSpeed); 
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}