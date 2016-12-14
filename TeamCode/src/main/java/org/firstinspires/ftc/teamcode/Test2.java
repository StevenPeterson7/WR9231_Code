package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/**
 * A test example of autonomous opmode programming using AutoLib classes.
 * Created by phanau on 12/14/15.
 */


@Autonomous(name="Test AutoLib", group ="Test")
//@Disabled
public class Test2 extends OpMode {

    AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;                  // true when the programmed sequence is done
    hardwareDeclare hw;

    public Test2() {
    }

    public void init() {
        // Get our hardware
        hw = new hardwareDeclare(this);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        mSequence.add(new AutoLib.MoveByTimeStep(hw.motors,-0.5,.75,true));

        mSequence.add(new AutoLib.TimedMotorStep(hw.liftMotors[2],1.0,2,true));

        mSequence.add(new AutoLib.TimedMotorStep(hw.liftMotors[1],1.0,2,true));

        mSequence.add(new AutoLib.TimedMotorStep(hw.liftMotors[2],1.0,2,true));
        
        // start out not-done
        bDone = false;
    }

    public void loop() {
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
    }

    public void stop() {
        telemetry.addData("stop() called", "");
    }
}
