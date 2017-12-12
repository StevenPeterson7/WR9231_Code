package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;

@Autonomous(name="MainRedAutonomousRatbot", group ="Autonomous")
//@Disabled
public class AutonomousRedMainRatbot extends OpMode {

    AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;        // true when the programmed sequence is done
    hardwareDeclare2 hw;
    SensorLib.PID mPID;

    float Kp = 0.035f;
    float Ki = 0.02f;
    float Kd = 0;
    float KiCutoff = 3.0f;



    public AutonomousRedMainRatbot() {
    }

    public void init() {
        // Get our hardware
        hw = new hardwareDeclare2(this);

        mPID = new SensorLib.PID(Kp,Ki,Kd,KiCutoff);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

      //  mSequence.add(new AutoLib.ServoStep(hw.whacker, 0.8));
        mSequence.add(new AutoLib.wait(1.0));
      //  mSequence.add(new AutoLib.knockJewelBlueRatbot(hw.mColorSensor, hw.motors, this));
        mSequence.add(new AutoLib.wait(5.0));

      //  mSequence.add(new AutoLib.ServoStep(hw.whacker, 0));
        mSequence.add(new AutoLib.wait(1.0));

        mSequence.add(new AutoLib.MoveByTimeStep(hw.motors,-0.5,1.0,true));





       // mSequence.add(new AutoLib.AzimuthTimedDriveStep(this,0,hw.mGyro,mPID,hw.motors,-.5f,.8f,true));
        //mSequence.add(new AutoLib.TimedMotorStep(hw.liftMotors[2],1.0,2,true));
        //mSequence.add(new AutoLib.TimedMotorStep(hw.liftMotors[1],1.0,2,true));
        //mSequence.add(new AutoLib.TimedMotorStep(hw.liftMotors[2],1.0,2,true));
        //mSequence.add(new AutoLib.MoveByEncoderStep(hw.motors, 1, ));
        //mSequence.add(new AutoLib.AzimuthTimedDriveStep(this,0,hw.mGyro,mPID,hw.motors,-.5f,.8f,true));

        //mSequence.add(new AutoLib.BeaconPushStep(hw.mColorSensor,0,hw.servos));
//        mSequence.add(new AutoLib.MoveByTimeStep(hw.motors,-0.5,.8,true));

        // start out not-done
        bDone = false;
    }

    public void loop() {
       // telemetry.addData("blue:", hw.mColorSensor.blue());
        //telemetry.addData("red:", hw.mColorSensor.red());

//        telemetry.addData("pos", hw.whacker.getPosition());

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
    }

    public void stop() {
        telemetry.addData("stop() called", "");
    }
}
