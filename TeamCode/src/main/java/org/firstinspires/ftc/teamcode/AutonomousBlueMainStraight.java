package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_FTC2017;

@Autonomous(name="StraightBlueAutonomous", group ="Autonomous")
//@Disabled
public class AutonomousBlueMainStraight extends AutonomousBlueMain {

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
    VuforiaLib_FTC2017 mVLib;               // Vuforia wrapper object used by Steps



    public AutonomousBlueMainStraight() {
    }

    public void init() {
        // Get our hardware
        hw = new hardwareDeclare(this);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        hw.imu.initialize(parameters);

        mVLib = new VuforiaLib_FTC2017();
        mVLib.init(this, null);


        mPID = new SensorLib.PID(Kp,Ki,Kd,KiCutoff);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();
       // AutoLib.MotorGuideStep guideStep  = new AutoLib.GoToCryptoBoxGuideStep(this, mVLib, "^b+", 0.175f);
      /*  mSequence.add(new AutoLib.ServoStep(hw.whacker, 0.45));
        mSequence.add(new AutoLib.wait(1.0));
        mSequence.add(new AutoLib.setColorB(hw.ColorSensor, this));
        mSequence.add(new AutoLib.wait(1.0));
        mSequence.add(new AutoLib.ServoStep(hw.whacker, 0.20));


        mSequence.add(new AutoLib.wait(1.0));
        mSequence.add(new AutoLib.knockJewelRed(hw.ColorSensor, hw.motors, this));
        mSequence.add(new AutoLib.wait(3.0));

        mSequence.add(new AutoLib.ServoStep(hw.whacker, 1));
        mSequence.add(new AutoLib.wait(1.5));
*/
      /*[  mSequence.add(new AutoLib.turnToGyroHeading(hw.motors, this, hw.imu, 100f));//turn 90 degrees to the left
        mSequence.add(new AutoLib.VuforiaGetMarkStep(this, mVLib, (AutoLib.SetMark)guideStep));
        mSequence.add(new AutoLib.wait(3));
        mSequence.add(new AutoLib.turnToGyroHeading(hw.motors, this, hw.imu, -15f));//turn 105 degrees to the right

        // make and add the Step that goes to the indicated Cryptobox bin
        mSequence.add(new AutoLib.GuidedTerminatedDriveStep(this, guideStep, null, hw.motors));
        // make and add a step that tells us we're done
        mSequence.add(new AutoLib.LogTimeStep(this,"Done!", 5));*/
       mSequence.add(new AutoLib.driveUntilCryptoColumn(this, mVLib, "^b+", 0.135f, 0, true, hw.imu, hw.motors));

        bDone = false;
    }
    @Override public void start()
    {

        // start Vuforia scanning
        mVLib.start();
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
