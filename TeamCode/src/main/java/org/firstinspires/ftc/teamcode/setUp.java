package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;

@Autonomous(name="SetUp", group ="SetUp")
//@Disabled
public class setUp extends OpMode {

    AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;                  // true when the programmed sequence is done
    hardwareDeclare hw;
    SensorLib.PID mPID;

    float Kp = 0.035f;
    float Ki = 0.02f;
    float Kd = 0;
    float KiCutoff = 3.0f;

    public setUp() {
    }

    public void init() {
        // Get our hardware
        hw = new hardwareDeclare(this);

        mPID = new SensorLib.PID(Kp,Ki,Kd,KiCutoff);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

       mSequence.add(new AutoLib.raiseLift(hw.glyphLift, 2500, this));
       mSequence.add(new AutoLib.ServoStep(hw.glyphLiftArms[0], 1));
        mSequence.add(new AutoLib.ServoStep(hw.glyphLiftArms[1], 1));

        bDone = false;
    }

    public void loop() {

        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
    }

    public void stop() {
        telemetry.addData("stop() called", "");
    }


}
