package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_FTC2017;

@Autonomous(name="tesAutonomous!!!!!!", group ="Autonomous")
//@Disabled
public class Autonomous2017test extends OpMode {

    public AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;                  // true when the programmed sequence is done
    hardwareDeclare hw;

    SensorLib.PID mPID;
    public int [] rgb = {0, 0, 0};
    public int d =0;
    float Kp = 0.035f;
    float Ki = 0.02f;
    float Kd = 0;
    float KiCutoff = 3.0f;
    int color=2;
    Orientation angles;
    VuforiaLib_FTC2017 mVLib;               // Vuforia wrapper object used by Steps
    boolean onTeamBlue;
    double movePower=0.25;
    public int targetColumn=3;




    public void init() {
        onTeamBlue=false;
        if(!onTeamBlue){
            movePower=-movePower;
        }

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
        hw.armTwist[0].setPosition(0);
        hw.armTwist[1].setPosition(0);

        mVLib = new VuforiaLib_FTC2017();
        mVLib.init(this, null);


        mPID = new SensorLib.PID(Kp,Ki,Kd,KiCutoff);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();


        //these functions need to be tested


        //mSequence.add(new AutoLib.wait(2));
        //target position needs to be found
        //mSequence.add(new AutoLib.alignWhacker(this, -80, mVLib, hw.motors));


        //mSequence.add(new AutoLib.MoveByTimeStep(hw.motors,movePower*1.75, 0.2, true));


        //this be sketch, test it


        //this needs to be tested and fine-tuned
        mSequence.add(new AutoLib.driveUntilCryptoColumn(new Autonomous2017(), mVLib, onTeamBlue ? "^b+" : "^r+", 0.15f, onTeamBlue, hw.imu, hw.motors, false));

        //these instructions might need to be switched

        //these need to be fine tuned

       // mSequence.add(new AutoLib.MoveByTimeStep(hw.motors, Math.abs(movePower), 0.5, false));




        bDone = false;
    }
    @Override public void start()
    {

        // start Vuforia scanning
        mVLib.start();
    }

    public void setTargetColumn(int tColumn){
        targetColumn=tColumn;
    }

    public void loop() {
        telemetry.addData("distance", d);
        telemetry.addData("blue:", hw.ColorSensor.blue()-rgb[2]);
        telemetry.addData("red:", hw.ColorSensor.red()-rgb[0]);
        telemetry.addData("power", hw.motors[0].getPower());
        telemetry.addData("important!!!0:", rgb[0]);
        telemetry.addData("important!!!2:", rgb[2]);
        telemetry.addData("target column", targetColumn);
        telemetry.addData("orientation", hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
    }

    public void stop() {
        telemetry.addData("stop() called", "");
        super.stop();
        mVLib.stop();
    }


}
