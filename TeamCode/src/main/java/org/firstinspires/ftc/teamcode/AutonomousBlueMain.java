package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="MainBlueAutonomous", group ="Autonomous")
//@Disabled
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
        mSequence.add(new AutoLib.wait(1.0));

        angles.firstAngle=0.25f;

        //mSequence.add(new AutoLib.turnByGyroHeading(hw.motors, this, hw.imu, angles));

       // mSequence.add(new AutoLib.identifyVuMark(this));

       // mSequence.add(new AutoLib.AzimuthTimedDriveStep(this,0,hw.mGyro,mPID,hw.motors,-.5f,.8f,true));
        //mSequence.add(new AutoLib.TimedMotorStep(hw.liftMotors[2],1.0,2,true));
        //mSequence.add(new AutoLib.TimedMotorStep(hw.liftMotors[1],1.0,2,true));
        //mSequence.add(new AutoLib.TimedMotorStep(hw.liftMotors[2],1.0,2,true));
        //mSequence.add(new AutoLib.MoveByEncoderStep(hw.motors, 1, ));
        //mSequence.add(new AutoLib.AzimuthTimedDriveStep(this,0,hw.mGyro,mPID,hw.motors,-.5f,.8f,true));

//        mSequence.add(new AutoLib.BeaconPushStep(hw.mColorSensor,0,hw.servos));
//        mSequence.add(new AutoLib.MoveByTimeStep(hw.motors,-0.5,.8,true));

        // start out not-done
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
