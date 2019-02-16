package frc.robot.simulator;

import com.snobot.simulator.ASimulator;
import com.snobot.simulator.module_wrapper.TestDigitalIOJni;
//import frc.robot.Robot;
import org.usfirst.frc5974.DeepSpace.Robot;

/**
 * A custom simulator. Currently this is empty, but if we
 * wanted to add custom mocking code, we could do it here.
 * See https://github.com/pjreiniger/SnobotSimExamples/blob/master/JavaWithI2CAndSPISims/src/snobot_sim/java/com/snobot/sim/Snobot2018Simulator.java
 * for an example
 */
public class RoboSimulator extends ASimulator {

    private Robot robot;

    TestDigitalIOJni digitalTest = new TestDigitalIOJni();

    public void create() {

    }

    @Override
    public void update() {
        super.update();
        digitalTest.testCreateDigitalIn();
    }

    public void setRobot(Robot aRobot) {
        this.robot = aRobot;
    }

}
