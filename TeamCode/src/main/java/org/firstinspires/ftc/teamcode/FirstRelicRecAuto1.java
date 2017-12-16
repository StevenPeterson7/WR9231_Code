/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Point;
import android.graphics.Rect;
import android.graphics.RectF;
import android.hardware.Camera;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.CameraLib;
//import org.firstinspires.ftc.teamcode._Libs.DistanceSensor;
import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_FTC2017;

import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;



//@Autonomous(name="FirstRelicRecAuto1", group ="Auto")
//@Disabled
public class FirstRelicRecAuto1 extends OpMode {

    boolean bDone;
    AutoLib.Sequence mSequence;             // the root of the sequence tree
    DcMotor mMotors[];                      // motors, some of which can be null: assumed order is fr, br, fl, bl
    BNO055IMU mImu;                       // gyro to use for heading information
    //SensorLib.CorrectedGyro mCorrGyro;      // gyro corrector object
    VuforiaLib_FTC2017 mVLib;               // Vuforia wrapper object used by Steps

    public void init() {}

    public void init(boolean bLookForBlue)
    {
        // get the hardware
        hardwareDeclare hw;

        hw=new hardwareDeclare(this);


        // get the motors: depending on the factory we created above, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        // assumed order is fr, br, fl, bl
        mMotors = hw.motors;

        // get hardware gyro
       // mImu = hw.imu;

        // wrap gyro in an object that calibrates it and corrects its output
       // mCorrGyro = new SensorLib.CorrectedGyro(mImu);
        //mCorrGyro.calibrate();

        // best to do this now, which is called from opmode's init() function
        mVLib = new VuforiaLib_FTC2017();
        mVLib.init(this, null);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();
        // make a step that guides the motion step by looking for a particular (red or blue) Cryptobox
        // it also implements the SetMark interface so VuforiaGetMarkStep can call it to tell which box to go for
        AutoLib.MotorGuideStep guideStep  = new AutoLib.GoToCryptoBoxGuideStep(this, mVLib, bLookForBlue ? "^b+" : "^r+", 0.175f);
        // make and add to the sequence the step that looks for the Vuforia marker and sets the column (Left,Center,Right)
        // the motion terminator step should look for

        mSequence.add(new AutoLib.VuforiaGetMarkStep(this, mVLib, (AutoLib.SetMark)guideStep));
        mSequence.add(new AutoLib.wait(3));
        // make and add the Step that goes to the indicated Cryptobox bin
        mSequence.add(new AutoLib.GuidedTerminatedDriveStep(this, guideStep, null, mMotors));
        // make and add a step that tells us we're done
        mSequence.add(new AutoLib.LogTimeStep(this,"Done!", 5));
    }

    @Override public void start()
    {
        // start out not-done
        bDone = false;

        // start Vuforia scanning
        mVLib.start();
    }

    @Override
    public void loop() {

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    @Override
    public void stop() {
        super.stop();
        mVLib.stop();     // make sure the Camera is released by Vuforia
    }

}

