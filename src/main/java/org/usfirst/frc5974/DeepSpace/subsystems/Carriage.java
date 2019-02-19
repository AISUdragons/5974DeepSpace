package org.usfirst.frc5974.DeepSpace.subsystems;

import org.usfirst.frc5974.DeepSpace.commands.Carry;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Carriage extends Subsystem{
    
    double grabSpeed = 1; //grabber/intake motor speed
    boolean hasBall = false;
    boolean intakeActive=false;
    boolean shootActive=false;

    //Carriage
	public VictorSP motorGrabL = new VictorSP(5);
	public VictorSP motorGrabR = new VictorSP(6);
    Controller controls = new Controller();
    //Variables
    public double speedModifier = .5; //change this to make lift move faster or slower
    public int targetLevel = 0; //level it's supposed to go to
    public double currentLevel = 0; //level it's currently at
    public double liftSpeed = 0; //the value we set motorLift to
    public int liftMode = 0; //0 is trigger, 1 is limit switch.

    //Lift
		public DigitalInput switchBottom = new DigitalInput(0); //TODO: Set limit switches to the correct ports
        public DigitalInput switchL1 = new DigitalInput(1);
        public DigitalInput switchL2 = new DigitalInput(2);
        public DigitalInput switchL3 = new DigitalInput(3);
        public DigitalInput switchTop = new DigitalInput(4);

        public VictorSP motorLift = new VictorSP(4);
        
    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Carry());
    }
}