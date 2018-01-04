package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="MainBlueAutonomous", group ="Autonomous")
@Disabled
public class AutonomousBlueMain extends OpMode {

    AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;                  // true when the programmed sequence is done
    hardwareDeclare hw;
    SensorLib.PID mPID;
    public int [] rgb = {0, 0, 0};

    float Kp = 0.035f;
    float Ki = 0.02f;
    float Kd = 0;
    float KiCutoff = 3.0f;
    int color=2;
    Orientation angles;
    int targetColumn=1;


    public AutonomousBlueMain() {
    }

    public void init() {
        // Get our hardware
        hw = new hardwareDeclare(this);

        mPID = new SensorLib.PID(Kp,Ki,Kd,KiCutoff);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        mSequence.add(new AutoLib.ServoStep(hw.whacker, 0.45));
        mSequence.add(new AutoLib.wait(1.0));
        mSequence.add(new AutoLib.setColorB(hw.ColorSensor, this));
        mSequence.add(new AutoLib.wait(1.0));
        mSequence.add(new AutoLib.ServoStep(hw.whacker, 0.20));


        mSequence.add(new AutoLib.wait(1.0));
        mSequence.add(new AutoLib.knockJewelRed(hw.ColorSensor, hw.motors, this));
        mSequence.add(new AutoLib.wait(3.0));

        mSequence.add(new AutoLib.ServoStep(hw.whacker, 1));
        mSequence.add(new AutoLib.wait(1.5));

        mSequence.add(new AutoLib.turnToGyroHeading(hw.motors, this, hw.imu, 90f));//turn 90 degrees to the left




        bDone = false;
    }

    public void loop() {
        telemetry.addData("blue:", hw.ColorSensor.blue()-rgb[2]);
        telemetry.addData("red:", hw.ColorSensor.red()-rgb[0]);
        telemetry.addData("power", hw.motors[0].getPower());
        telemetry.addData("important!!!0:", rgb[0]);
        telemetry.addData("important!!!2:", rgb[2]);
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
    }

    public void stop() {
        telemetry.addData("stop() called", "");
    }


}
