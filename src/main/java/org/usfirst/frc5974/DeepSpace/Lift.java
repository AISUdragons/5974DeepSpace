package org.usfirst.frc5974.DeepSpace;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

import org.usfirst.frc5974.DeepSpace.Controller;

public class Lift{
    VictorSP motorLift = new VictorSP(4);
    VictorSP motorGrabLeft = new VictorSP(5);
    VictorSP motorGrabRight = new VictorSP(6);

    Controller controls = new Controller();
    Joystick keyboard = new Joystick(1); //Keyboard (for simulation only)

    //Encoder constructor
    int channelA = 0;
    int channelB = 1;
    Encoder encoder = new Encoder(channelA,channelB);

    //Variables
    double speedModifier = .5; //change this to make lift move faster or slower
    int targetLevel = 0; //level it's supposed to go to
    double currentLevel = 0; //level it's currently at
    int[] heights = {0,1,2,3}; //heights for each goal (only used in encoder mode)
    double liftSpeed = 0; //the value we set motorLift to
    int liftMode = 0; //0 is trigger, 1 is switch, 2 is encoder. We can probably remove the extra code once we know which one we're using.
    double grabSpeed = 1;
    boolean hasBall = false;
    boolean intakeActive=false;
    boolean shootActive=false;
    boolean climberUp = true;

    public void updateLevel(){
        //Update bumper - user input for which level to go to.
        if(controls.runOnce(controls.bumperR,controls.pairBumperR)&&targetLevel<3){
            //if bumper R is pressed and target level is less than 3, increase target level
            targetLevel++;
            System.out.println("Target++");
        }
        else if(controls.runOnce(controls.bumperL,controls.pairBumperL)&&targetLevel>0){
            //if bumper L is pressed and target level is more than 0, decrease target level
            targetLevel--;
            System.out.println("Target--");
        }

        //Kill if lift hits top or bottom limit switches.
        if(controls.switchBottom){
            liftSpeed=Math.max(0,liftSpeed); //We can't just set it to 0, because the limit switch will continue being held down, disabling the motor for the rest of the game.
            //This ensures the lift speed will be positive.
            System.out.println("bottom");
        }
        if(controls.switchTop){
            liftSpeed = Math.min(0,liftSpeed); //See above comments; this ensures lift speed will be negative.
            System.out.println("top");
        }
        
        //Update limit switches for every level
        if(controls.switchL1){ //If the limit switch for L1 is hit:
            System.out.println("1");
            if(currentLevel<1){ //If the current level is less than L1:
                currentLevel++; //Increase current level
            }
            else if(currentLevel>1){ //If the current level is greater than L1:
                currentLevel--; //Decrease current level.
            }
        }
        if(controls.switchL2){ //Same as above.
            System.out.println("2");
            if(currentLevel<2){
                currentLevel++;
            }
            else if(currentLevel>2){
                currentLevel--;
            }
        }
        if(controls.switchL3){
            System.out.println("3");
            if(currentLevel<3){
                currentLevel++;
            }
            else if(currentLevel>3){
                currentLevel--;
            }
        }
    }

    public double triggerLift(double down,double up) { //This is what we'll use if we can't get limit switches or encoders set up - completely user controlled.
        if(down>0){
            //Move up if right trigger is pressed and left isn't
            System.out.println(down);
            return -up*speedModifier;
        }
        else if(up<0){
            //Move down if left trigger is pressed and right isn't
            System.out.println(up);
            return -down*speedModifier;
        }
        else{
            return 0;
        }
    }

    public void limitLift(){ //Operate lift based on limit switches
        updateLevel();
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
            //triggerLift();
        }
        else if(liftMode==1){
            limitLift();
        }
        else if(liftMode==2){
            encoderLift();
        }

        motorLift.set(liftSpeed); 

         //Theoretically, this will take in balls if it's at the bottom level, and shoot if it's at higher levels.
         if(controls.buttonX&&currentLevel==0){
            intake();
        }
        else if(controls.buttonX&&currentLevel>0){
            shoot();
        }
        else if(!controls.buttonX){
            motorGrabLeft.set(0);
            motorGrabRight.set(0);
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
                motorGrabLeft.set(0);
                motorGrabRight.set(0);
                intakeActive=false;
                hasBall=true;
            }
            if(shootActive){
                motorGrabLeft.set(0);
                motorGrabRight.set(0);
                shootActive=false;
                hasBall=false;
            }
        }
    }

    public void intake(){
        motorGrabLeft.set(grabSpeed);
        motorGrabRight.set(-grabSpeed);
    }

    public void shoot(){
        motorGrabLeft.set(-grabSpeed);
        motorGrabRight.set(grabSpeed);
    }
}

//I deleted basically everything that was here previously. If you want it back, check Github :P