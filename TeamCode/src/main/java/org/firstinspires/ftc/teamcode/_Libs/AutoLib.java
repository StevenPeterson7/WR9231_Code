package org.firstinspires.ftc.teamcode._Libs;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Autonomous2017;
import org.firstinspires.ftc.teamcode.AutonomousBlueMain;
import org.firstinspires.ftc.teamcode.AutonomousRedMain;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Created by phanau on 12/14/15.
 */

// a library of classes that support autonomous opmode programming
public class AutoLib {

    // the base class from which everything else derives.
    // each action in an autonomous sequence is a Step of some kind.
    // a Step may be simple (like run a Motor) or a composite of several Steps which
    // are either run sequentially or in parallel (see Sequences below).
    static public abstract class Step {

        int mLoopCount;     // keeps count of how many times loop() has been called on this Step

        protected Step() {
            mLoopCount = 0;
        }

        // returns true iff called from the first call to loop() on this Step
        boolean firstLoopCall() {
            boolean flc = (mLoopCount == 1);    // assume this is called AFTER super.loop()
            mLoopCount++;
            return flc;
        }

        // run the next time-slice of the Step; return true when the Step is completed
        boolean loop() {
            mLoopCount++;       // increment the loop counter
            return false;
        }

    }

    // ------------------ some implementations of Sequence constructs -------------------------

    // base class for Sequences that perform multiple Steps, either sequentially or concurrently
    static public abstract class Sequence extends Step {
        protected ArrayList<Step> mSteps;  // expandable array containing the Steps in the Sequence

        protected Sequence() {
            mSteps = new ArrayList<Step>(10);   // create the array with an initial capacity of 10
        }

        // add a Step to the end of the Sequence
        public Step add(Step step) {
            mSteps.add(step);
            return this;        // allows daisy-chaining of calls
        }

        // add a Step to the beginning of the Sequence - used for a control steps that
        // needs to run BEFORE the e.g. motor steps it controls
        public Step preAdd(Step step) {
            mSteps.add(0, step);
            return this;
        }

        // run the next time-slice of the Sequence; return true when the Sequence is completed.
        public boolean loop() {
            super.loop();
            return false;
        }

    }

    // a Sequence that performs its constituent Steps sequentially
    static public class LinearSequence extends Sequence {
        int mIndex;     // index of currently active Step

        public LinearSequence() {
            mIndex = 0;     // start at the beginning
        }

        // run the current Step of the Sequence until it completes, then the next Step and
        // the next, etc., etc. until the last Step completes, at which point the Sequence
        // returns complete.
        public boolean loop() {
            super.loop();
            if (mIndex < mSteps.size()) {       // if this Sequence is not completed ...
                if (mSteps.get(mIndex).loop())  // if this Step is complete, move to the next Step
                    mIndex++;
            }
            return (mIndex >= mSteps.size());   // return true when last Step completes
        }

    }


    // a Sequence that performs its constituent Steps concurrently
    static public class ConcurrentSequence extends Sequence {

        public ConcurrentSequence() {
        }

        // run all the Steps in the Sequence "concurrently" -- i.e. run the loop() function of
        // each of the Steps each time loop() is called. When ALL the Steps report that they
        // are done, then this Sequence is done.
        public boolean loop() {
            super.loop();
            boolean bDone = true;
            for (Step s : mSteps)
                bDone &= s.loop();      // "done" means ALL Steps are done
            return bDone;
        }

    }


    // ------------------ some implementations of primitive Steps ----------------------------

    // a simple Step that just logs its existence for a given number of loop() calls
    // really just for testing sequencer stuff, not for actual use on a robot.
    static public class LogCountStep extends Step {
        OpMode mOpMode;     // needed so we can log output
        String mName;       // name of the output field
        int mCount;         // current loop count of this Step

        public LogCountStep(OpMode opMode, String name, int loopCount) {
            mOpMode = opMode;
            mName = name;
            mCount = loopCount;
        }

        public boolean loop() {
            super.loop();

            // log some info about this Step
            if (mCount > 0) {
                mOpMode.telemetry.addData(mName, "count = " + mCount);
                mCount--;

                // wait a bit so we can see the displayed info ...
                try {
                    Thread.sleep(500);
                } catch (Exception e) {
                }
            } else
                mOpMode.telemetry.addData(mName, "done");


            // return true when count is exhausted
            return (mCount <= 0);
        }

    }


    // a simple Step that just logs its existence for a given length of time
    static public class LogTimeStep extends Step {
        OpMode mOpMode;     // needed so we can log output
        String mName;       // name of the output field
        Timer mTimer;       // Timer for this Step

        public LogTimeStep(OpMode opMode, String name, double seconds) {
            mOpMode = opMode;
            mName = name;
            mTimer = new Timer(seconds);
        }

        public boolean loop() {
            super.loop();

            // start the Timer on our first call
            if (firstLoopCall())
                mTimer.start();

            // log some info about this Step
            if (!mTimer.done())        // appears to cycle here at about 3ms/loop
                mOpMode.telemetry.addData(mName, "time = " + mTimer.remaining());
            else
                mOpMode.telemetry.addData(mName, "done");

            // return true when time is exhausted
            return (mTimer.done());
        }

    }

    // interface for setting the current power of either kind of MotorStep
    interface SetPower {
        public void setPower(double power);
    }

    // a Step that runs a DcMotor at a given power, for a given time
    static public class TimedMotorStep extends Step implements SetPower {
        Timer mTimer;
        DcMotor mMotor;
        double mPower;
        boolean mStop;          // stop motor when count is reached

        public TimedMotorStep(DcMotor motor, double power, double seconds, boolean stop) {
            mMotor = motor;
            mPower = power;
            mTimer = new Timer(seconds);
            mStop = stop;
        }

        // for dynamic adjustment of power during the Step
        public void setPower(double power) {
            mPower = power;
        }

        public boolean loop() {
            super.loop();

            // start the Timer and start the motor on our first call
            if (firstLoopCall()) {
                mTimer.start();
                mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mMotor.setPower(mPower);
            }

            // run the motor at the requested power until the Timer runs out
            boolean done = mTimer.done();
            if (done && mStop)
                mMotor.setPower(0);
            else
                mMotor.setPower(mPower);        // update power in case it changed

            return done;
        }

    }


    // a Step that runs a DcMotor at a given power, for a given encoder count
    static public class EncoderMotorStep extends Step implements SetPower {
        DcMotor mMotor;    // motor to control
        double mPower;          // power level to use
        int mEncoderCount;      // target encoder count
        int mState;             // internal state machine state
        boolean mStop;          // stop motor when count is reached

        public EncoderMotorStep(DcMotor motor, double power, int count, boolean stop) {
            mMotor = motor;
            mPower = power;
            mEncoderCount = count;
            mState = 0;
            mStop = stop;
        }

        // for dynamic adjustment of power during the Step
        public void setPower(double power) {
            mPower = power;
        }

        public boolean loop() {
            super.loop();

            boolean done = false;

            // we need a little state machine to make the encoders happy
            if (firstLoopCall()) {
                // set up the motor on our first call
                mMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mMotor.setTargetPosition(mMotor.getCurrentPosition() + mEncoderCount);
                mMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                mMotor.setPower(mPower);
                mState++;
            }

            // the rest of the time, just update power and check to see if we're done
            done = !mMotor.isBusy();
            if (done && mStop)
                mMotor.setPower(0);     // optionally stop motor when target reached
            else
                mMotor.setPower(mPower);        // update power in case it changed

            return done;
        }

    }
    static public class wait extends Step {
        Timer mTimer;
        boolean done=false;
        public wait (double t){
            mTimer=new Timer (t);
        }
        public boolean loop() {
            super.loop();

            if (firstLoopCall()){
               mTimer.start();
            }

            done = mTimer.done();

            return done;
        }

    }

    static public class setColorR extends Step {
    Timer t = new Timer(0.25);
    boolean done;
    public setColorR (ColorSensor colorSensor, AutonomousRedMain op) {

        op.rgb[0] = colorSensor.red();
        op.rgb[1] = colorSensor.green();
        op.rgb[2] = colorSensor.blue();
    }
    public boolean loop() {
        super.loop();

        if (firstLoopCall()){
            t.start();
        }

        done = t.done();
        return done;
    }

}

    static public class setColorB extends Step {
    Timer t = new Timer(0.25);
    boolean done;
    public setColorB (ColorSensor colorSensor, AutonomousBlueMain op) {

        op.rgb[0]=colorSensor.red();
        op.rgb[1]=colorSensor.green();
        op.rgb[2]=colorSensor.blue();
    }
    public boolean loop() {
        super.loop();

        if (firstLoopCall()){
            t.start();
        }

        done = t.done();
        return done;
    }

}
static public class setColor extends Step{
        Timer t=new Timer (0.25);
        boolean done;
        public setColor (ColorSensor colorSensor, Autonomous2017 op){
            op.rgb[0]=colorSensor.red();
            op.rgb[1]=colorSensor.green();
            op.rgb[2]=colorSensor.blue();
        }
    public boolean loop() {
        super.loop();

        if (firstLoopCall()){
            t.start();
        }

        done = t.done();
        return done;
    }

}

    int testColor( ColorSensor colorSensor){
        if (colorSensor.red()>= colorSensor.blue()*1.25){


            return 0;
        }
        else if (colorSensor.blue() >= colorSensor.red()*1.25){

            return 1;
        }
        else return 2;
    }
    static public class knockJewel extends Step{
        ColorSensor colorSensor;
        DcMotor [] motors;
        Timer mTimer = new Timer(0.35);
        Timer nTimer = new Timer (0.55);
        boolean done = false;
        boolean firstLoopStart=true;
        boolean secondLoopStart = true;
        int color=3;
        private Autonomous2017 mOpMode;                             // needed so we can log output (may be null)
        boolean onBlueTeam;
        double power = 0.25;
        public knockJewel(ColorSensor cs, DcMotor [] m, Autonomous2017 op, boolean blueTeam) {

            colorSensor = cs;
            motors = m;
            mOpMode = op;
            onBlueTeam=blueTeam;

        }
        public int ColorTest() {

            if (colorSensor.red()-mOpMode.rgb[0]>= (colorSensor.blue()-mOpMode.rgb[2])*1.25){

                mOpMode.telemetry.addData("color: ", "red");
                return 0;
            }
            else if (colorSensor.blue()-mOpMode.rgb[2] >= (colorSensor.red()-mOpMode.rgb[0])*1.25){
                mOpMode.telemetry.addData("color: ", "blue");
                return 1;
            }
            else return 2;
        }
        public boolean loop() {
            super.loop();

            if (firstLoopStart){
                if(!onBlueTeam){
                    power=-power;
                }

                firstLoopStart=false;
                color=ColorTest();
                //0-red
                //1-blue
                //2-indeterminate
                if (color == 0){

                    motors[0].setPower(-power);
                    motors[1].setPower(-power);
                    motors[2].setPower(-power);
                    motors[3].setPower(-power);
                    // mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, .5, 1.3, true));

                    mTimer.start();
                }
                else if (color ==1) {

                    motors[0].setPower(power);
                    motors[1].setPower(power);
                    motors[2].setPower(power);
                    motors[3].setPower(power);
                    // mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, .5, 0.9, true));


                    mTimer.start();
                }else if(color == 2){
                    mTimer.start();
                }
            }

            if(mTimer.done()&&secondLoopStart){
                secondLoopStart=false;
                if (color == 1){
                    motors[0].setPower(-power);
                    motors[1].setPower(-power);

                    motors[2].setPower(-power);
                    motors[3].setPower(-power);
                    //mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, -.5, 0.9, true));

                    nTimer.start();
                }
                else if (color ==0){
                    motors[0].setPower(power);
                    motors[1].setPower(power);
                    motors[2].setPower(power);
                    motors[3].setPower(power);
                    //mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, -.5, 1.3, true));

                    nTimer.start();
                }
                else if (color==2){
                    nTimer.start();
                    //mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, -0.5, 1.0, true));
                }
            }

            done = nTimer.done();

            if (done && mTimer.done()) {
                motors[0].setPower(0);
                motors[1].setPower(0);
                motors[2].setPower(0);
                motors[3].setPower(0);
                return true;
            }
            mOpMode.telemetry.addData("mTimer: ", mTimer.done());
            mOpMode.telemetry.addData("nTimer: ", nTimer.done());

            return false;
        }


    }
    static public class knockJewelRed extends Step {
        ColorSensor colorSensor;
        DcMotor [] motors;
        Timer mTimer = new Timer(0.35);
        Timer nTimer = new Timer (0.55);
        boolean done = false;
        boolean firstLoopStart=true;
        boolean secondLoopStart = true;
        int color=3;
        private AutonomousBlueMain mOpMode;                             // needed so we can log output (may be null)
        public knockJewelRed(ColorSensor cs, DcMotor [] m, AutonomousBlueMain op) {

            colorSensor = cs;
            motors = m;
            mOpMode = op;

        }
        public int ColorTest() {

            if (colorSensor.red()-mOpMode.rgb[0]>= (colorSensor.blue()-mOpMode.rgb[2])*1.25){

                mOpMode.telemetry.addData("color: ", "red");
                return 0;
            }
            else if (colorSensor.blue()-mOpMode.rgb[2] >= (colorSensor.red()-mOpMode.rgb[0])*1.25){
                mOpMode.telemetry.addData("color: ", "blue");
                return 1;
            }
            else return 2;
        }
        public boolean loop() {
            super.loop();

            if (firstLoopStart){
                firstLoopStart=false;
                color=ColorTest();
                //0-red
                //1-blue
                //2-indeterminate
                if (color == 0){

                    motors[0].setPower(-0.25);
                    motors[1].setPower(-0.25);
                    motors[2].setPower(-0.25);
                    motors[3].setPower(-0.25);
                   // mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, .5, 1.3, true));


                    mTimer.start();
                }
                else if (color ==1) {

                    motors[0].setPower(0.25);
                    motors[1].setPower(0.25);
                    motors[2].setPower(0.25);
                    motors[3].setPower(0.25);
                   // mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, .5, 0.9, true));


                    mTimer.start();
                }else if(color == 2){
                    mTimer.start();
                }
            }

            if(mTimer.done()&&secondLoopStart){
                secondLoopStart=false;
                if (color == 1){
                    motors[0].setPower(-0.25);
                    motors[1].setPower(-0.25);

                    motors[2].setPower(-0.25);
                    motors[3].setPower(-0.25);
                    //mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, -.5, 0.9, true));

                    nTimer.start();
                }
                else if (color ==0){
                    motors[0].setPower(0.25);
                    motors[1].setPower(0.25);
                    motors[2].setPower(0.25);
                    motors[3].setPower(0.25);
                    //mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, -.5, 1.3, true));

                    nTimer.start();
                }
                else if (color==2){
                    nTimer.start();
                    //mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, -0.5, 1.0, true));
                }
            }

            done = nTimer.done();

            if (done && mTimer.done()) {
                motors[0].setPower(0);
                motors[1].setPower(0);
                motors[2].setPower(0);
                motors[3].setPower(0);
                return true;
            }
            mOpMode.telemetry.addData("mTimer: ", mTimer.done());
            mOpMode.telemetry.addData("nTimer: ", nTimer.done());

            return false;
        }


    }
    static public class knockJewelBlue extends Step {
        ColorSensor colorSensor;
        DcMotor [] motors;
        Timer mTimer = new Timer(0.35);
        Timer nTimer = new Timer (0.55);
        boolean done = false;
        int color=3;
        boolean firstLoopStart=true;
        boolean secondLoopStart =true;
        private AutonomousRedMain mOpMode;                             // needed so we can log output (may be null)
        public knockJewelBlue(ColorSensor cs, DcMotor [] m, AutonomousRedMain op) {

            colorSensor = cs;
            motors = m;
            mOpMode=op;

        }
        public int ColorTest() {

            if (colorSensor.red()-mOpMode.rgb[0]>= (colorSensor.blue()-mOpMode.rgb[2])*1.25){

                mOpMode.telemetry.addData("color: ", "red");
                return 0;
            }
            else if (colorSensor.blue()-mOpMode.rgb[2] >= (colorSensor.red()-mOpMode.rgb[0])*1.25){
                mOpMode.telemetry.addData("color: ", "blue");
                return 1;
            }
            else return 2;
        }
        public boolean loop() {
            super.loop();
            if (firstLoopStart){
                color=ColorTest();
                //0-red
                //1-blue
                //2-indeterminate
                firstLoopStart=false;
                if (color == 1){
                    motors[0].setPower(-0.25);
                    motors[1].setPower(-0.25);

                    motors[2].setPower(-0.25);
                    motors[3].setPower(-0.25);
                  //  mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, -.5, 0.9, true));

                    mTimer.start();
                }
                else if (color ==0){
                    motors[0].setPower(0.25);
                    motors[1].setPower(0.25);
                    motors[2].setPower(0.25);
                    motors[3].setPower(0.25);
                 //   mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, -.5, 1.3, true));

                    mTimer.start();
                }
                else if (color==2){
                    mTimer.start();
                   // mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, -0.5, 1.0, true));
                }
            }

            if(mTimer.done()&&secondLoopStart){
                secondLoopStart=false;
                if (color == 1){
                    motors[0].setPower(0.25);
                    motors[1].setPower(0.25);

                    motors[2].setPower(0.25);
                    motors[3].setPower(0.25);
                    //mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, -.5, 0.9, true));

                    nTimer.start();
                }
                else if (color ==0){
                    motors[0].setPower(-0.25);
                    motors[1].setPower(-0.25);
                    motors[2].setPower(-0.25);
                    motors[3].setPower(-0.25);
                    //mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, -.5, 1.3, true));

                    nTimer.start();
                }
                else if (color==2){
                    nTimer.start();
                    //mOpMode.mSequence.add(new AutoLib.MoveByTimeStep(motors, -0.5, 1.0, true));
                }
            }

            done = nTimer.done();


            if (done && mTimer.done()) {
                motors[0].setPower(0);
                motors[1].setPower(0);
                motors[2].setPower(0);
                motors[3].setPower(0);
                return true;
            }
            mOpMode.telemetry.addData("mTimer: ", mTimer.done());
            mOpMode.telemetry.addData("nTimer: ", nTimer.done());


            return false;
        }
    }


    static public class knockJewelRedRatbot extends Step {
        ModernRoboticsI2cColorSensor mColorSensor;
        DcMotor [] motors;
        Timer mTimer = new Timer(0.25);
        boolean done = false;
        private OpMode mOpMode;
        // needed so we can log output (may be null)
        public knockJewelRedRatbot(ModernRoboticsI2cColorSensor cs, DcMotor [] m, OpMode op) {

            mColorSensor = cs;
            motors = m;
            mOpMode=op;

        }
        public int ColorTest() {

            if (mColorSensor.red()>= mColorSensor.blue()*2){

                mOpMode.telemetry.addData("color: ", "red");
                return 0;
            }
            else if (mColorSensor.blue() >= mColorSensor.red() *2){
                mOpMode.telemetry.addData("color: ", "blue");
                return 1;
            }
            else return 2;
        }
        public boolean loop() {
            super.loop();

            if (firstLoopCall()){
                if (ColorTest() == 0){
                    motors[0].setPower(-0.25);
                    motors[1].setPower(-0.25);
                    motors[2].setPower(-0.25);
                    motors[3].setPower(-0.25);

                    mTimer.start();
                }
                else if (ColorTest() ==1) {
                    motors[0].setPower(0.25);
                    motors[1].setPower(0.25);
                    motors[2].setPower(0.25);
                    motors[3].setPower(0.25);

                    mTimer.start();
                }else{
                    mTimer.start();
                }
            }

            done = mTimer.done();

            if (done == true) {
                motors[0].setPower(0);
                motors[1].setPower(0);
                motors[2].setPower(0);
                motors[3].setPower(0);
            }

            return done;
        }


    }
    static public class knockJewelBlueRatbot extends Step {
        ModernRoboticsI2cColorSensor mColorSensor;
        DcMotor [] motors;
        Timer mTimer = new Timer(0.25);
        boolean done = false;
        private OpMode mOpMode;                             // needed so we can log output (may be null)
        public knockJewelBlueRatbot(ModernRoboticsI2cColorSensor cs, DcMotor [] m, OpMode op) {

            mColorSensor = cs;
            motors = m;
            mOpMode=op;

        }
        public int ColorTest() {

            if (mColorSensor.red()>= mColorSensor.blue()*2){

                mOpMode.telemetry.addData("color: ", "red");
                return 0;
            }
            else if (mColorSensor.blue() >= mColorSensor.red() *2){
                mOpMode.telemetry.addData("color: ", "blue");
                return 1;
            }
            else return 2;
        }
        public boolean loop() {
            super.loop();

            if (firstLoopCall()){
                if (ColorTest() == 1){
                    motors[0].setPower(-0.25);
                    motors[1].setPower(-0.25);
                    motors[2].setPower(-0.25);
                    motors[3].setPower(-0.25);

                    mTimer.start();
                }
                else if (ColorTest() ==0){
                    motors[0].setPower(0.25);
                    motors[1].setPower(0.25);
                    motors[2].setPower(0.25);
                    motors[3].setPower(0.25);

                    mTimer.start();
                }
            }

            done = mTimer.done();

            if (done == true) {
                motors[0].setPower(0);
                motors[1].setPower(0);
                motors[2].setPower(0);
                motors[3].setPower(0);
            }

            return done;
        }
    }





























static public class raiseLift extends Step{
        DcMotor [] glyphLift;
        boolean done = false;
        private OpMode mOpMode;
        int targetHeight;
        int startHeight;

        public raiseLift(DcMotor [] lift, int h, OpMode op){
            glyphLift=lift;
            targetHeight=h;
            startHeight=glyphLift[1].getCurrentPosition();
            mOpMode=op;
        }
    public boolean loop() {
        super.loop();

        if(glyphLift[1].getCurrentPosition()-startHeight < targetHeight-5){
            glyphLift[1].setPower(0.15);
        }else if(glyphLift[1].getCurrentPosition()-startHeight > targetHeight+5){
            glyphLift[1].setPower(-0.15);
        }else{
            glyphLift[1].setPower(0);
            return true;
        }


        return false;
    }
}

static public class placeGlyphFromSide extends Step{
        private OpMode mOpMode;
        Servo placer;
        Timer mTimer=new Timer(1.5);

        public placeGlyphFromSide( OpMode op, Servo p){
            mOpMode=op;
            placer=p;
        }

        public boolean loop() {
            super.loop();
            if(firstLoopCall()){
                mTimer.start();
                placer.setPosition(0.2);

            }
            if(mTimer.done()){
                placer.setPosition(1);
                return true;
            }
            return false;
        }

}

static public class pickUpGlyph extends Step{
    private Autonomous2017 mOpMode;
    DcMotor [] glyphLift;
    DcMotor [] motors;
    Servo [] arms;
    Timer mTimer=new Timer(0.25);
    boolean firstLoopDone=false;
    int targetHeight;
    int startHeight;
    double power;
    public pickUpGlyph(Autonomous2017 op,DcMotor [] gl, DcMotor [] m, Servo [] a, double p, int t){
        mOpMode=op;
        glyphLift=gl;
        motors=m;
        targetHeight=t;
        startHeight=glyphLift[1].getCurrentPosition();
        arms=a;
        power=p;
    }
    public boolean loop(){
        super.loop();
        if(firstLoopCall()){
            mTimer.start();
            motors[0].setPower(power);
            motors[1].setPower(power);
            motors[2].setPower(power);
            motors[3].setPower(power);
            arms[0].setPosition(1);
            arms[1].setPosition(1);

        }
        if(mTimer.done()){
            firstLoopDone=false;
            motors[0].setPower(0);
            motors[1].setPower(0);
            motors[2].setPower(0);
            motors[3].setPower(0);

        }
        if(firstLoopDone){
            if(glyphLift[1].getCurrentPosition()-startHeight < targetHeight-5){
                glyphLift[1].setPower(0.15);
            }else if(glyphLift[1].getCurrentPosition()-startHeight > targetHeight+5){
                glyphLift[1].setPower(-0.15);
            }else{
                glyphLift[1].setPower(0);
                arms[0].setPosition(0);
                arms[1].setPosition(0);
                return true;
            }

        }
        return false;
    }
}
static public class alignWhacker extends Step{

}
static public class placeGlyph extends Step{
    private Autonomous2017 mOpMode;

    Servo [] arms;
    Timer mTimer=new Timer(0.35);

    public placeGlyph( Autonomous2017 op,Servo [] a){
        mOpMode=op;

        arms=a;

    }
    public boolean loop(){
        super.loop();
        if(firstLoopCall()){
            mTimer.start();
            arms[0].setPosition(1);
            arms[1].setPosition(1);

        }
        if(mTimer.done()){
            return true;

        }

        return false;
    }

}
/*static public class pickUpGlyph extends ConcurrentSequence {
        DcMotor [] motors;
        boolean done=false;
        private OpMode mOpMode;
        DcMotor [] glyphLift;
        Servo [] arms;
        raiseLift rLift;

        public pickUpGlyph(DcMotor [] m, OpMode op, DcMotor [] lift, Servo [] a){
            motors = m;
            mOpMode = op;
            glyphLift = lift;
            arms = a;
            ServoStep moveL=new ServoStep(arms[0], 1);
            ServoStep moveR=new ServoStep(arms[1], 1);
            raiseLift raiseL=new raiseLift(glyphLift, -250, mOpMode);
            this.add(moveL);
            this.add(moveR);
            this.add(raiseL);
            this

        }

}
*/





    static public class turnToGyroHeading extends Step {
        BNO055IMU mIMU;
        DcMotor [] motors;
        boolean done = false;
        private OpMode mOpMode;                             // needed so we can log output (may be null)
        float mTargetHeading;// -180 to 180
        double motorPower=0.4;

        public turnToGyroHeading( DcMotor [] m, OpMode op, BNO055IMU imu, float targetHeading) {

            motors = m;
            mOpMode = op;
            mIMU = imu;
            mTargetHeading = targetHeading;

        }

        public void turnLeft(){
            motors[0].setPower(motorPower);
            motors[1].setPower(motorPower);
            motors[2].setPower(-motorPower);
            motors[3].setPower(-motorPower);


        }
        public void turnRight(){
            motors[0].setPower(-motorPower);
            motors[1].setPower(-motorPower);
            motors[2].setPower(motorPower);
            motors[3].setPower(motorPower);
        }

        public boolean loop() {
            super.loop();

            if (firstLoopCall()){


            }
            if(mIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > mTargetHeading + 1) {
                turnLeft();

            }else if(mIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < mTargetHeading - 1){
                turnRight();

            }else{
                done=true;
            }

            if (done == true) {
                motors[0].setPower(0);
                motors[1].setPower(0);
                motors[2].setPower(0);
                motors[3].setPower(0);
            }

            return done;
        }


    }


    static public class identifyVuMark extends Step {
        VuforiaLib_FTC2017 mVLib;
        DcMotor [] motors;
        boolean done = true;
        private Autonomous2017 mOpMode;                             // needed so we can log output (may be null)
        Timer mTimer=new Timer(3);
        int targetColumn;

        boolean blue;
        public identifyVuMark(Autonomous2017 op, DcMotor [] m, VuforiaLib_FTC2017 VLib, boolean b) {
            motors = m;
            mOpMode = op;
            mVLib = VLib;

            blue=b;
        }
        public boolean loop() {
            super.loop();
            mVLib.loop();


            if (firstLoopCall()) {
                mTimer.start();

            }
            if(targetColumn==0){
                RelicRecoveryVuMark vuMark = mVLib.getVuMark();

                switch (vuMark){
                    case LEFT:
                        targetColumn=1;
                        if(!blue){
                            targetColumn=3;
                        }
                        motors[0].setPower(0);
                        motors[1].setPower(0);
                        motors[2].setPower(0);
                        motors[3].setPower(0);
                        break;
                    case CENTER:
                        targetColumn=2;
                        if(!blue){
                            targetColumn=2;
                        }
                        motors[0].setPower(0);
                        motors[1].setPower(0);
                        motors[2].setPower(0);
                        motors[3].setPower(0);
                        break;
                    case RIGHT:
                        targetColumn=3;
                        if(!blue){
                            targetColumn=1;
                        }
                        motors[0].setPower(0);
                        motors[1].setPower(0);
                        motors[2].setPower(0);
                        motors[3].setPower(0);
                        break;
                    case UNKNOWN:
                        targetColumn=0;
                        motors[0].setPower(-0.15);
                        motors[1].setPower(-0.15);
                        motors[2].setPower(-0.15);
                        motors[3].setPower(-0.15);
                        break;

                }

            }else{
                mOpMode.targetColumn=targetColumn;
                return true;
            }
            if(mTimer.done()){
                return true;
            }


            return false;
        }
    }
    /**
     * This OpMode uses a Step that uses the VuforiaLib_FTC2017 library to determine
     * which column of the shelves to fill first, then
     * moves the robot under gyro control while using the camera to look for the
     * correct Cryptobox to stop at.
     */

// define an interface through which a Step (or anything else) can be told the
// identity of the Vuforia target that we should use
    public interface SetMark {
        public void setMark(String s);
    }

    static public class VuforiaGetMarkStep extends AutoLib.Step {

        VuforiaLib_FTC2017 mVLib;
        OpMode mOpMode;
        SetMark mSMStep;

        public VuforiaGetMarkStep(OpMode opMode, VuforiaLib_FTC2017 VLib, SetMark step) {
            mOpMode = opMode;
            mVLib = VLib;
            mSMStep = step;
        }

        public boolean loop() {
            super.loop();
            mVLib.loop();       // update recognition info
            RelicRecoveryVuMark vuMark = mVLib.getVuMark();
            boolean found = (vuMark != RelicRecoveryVuMark.UNKNOWN);
            if (found) {
                // Found an instance of the template -- tell "MoveTo.. step which one
                mSMStep.setMark(vuMark.toString());

            }
            return found;       // done?
        }
    }

    static public class BlueFilter implements CameraLib.Filter {
        public int map(int hue) {
            // map 4 (cyan) to 5 (blue)
            if (hue == 4)
                return 5;
            else
                return hue;
        }
    }

    // simple data class containing info about image of one column of cryptobox
    static public class ColumnHit {
        int mStart;
        int mEnd;
        public ColumnHit(int start, int end) {
            mStart = start;  mEnd = end;
        }
        public int start() { return mStart; }
        public int end() { return mEnd; }
        public int mid() { return (mStart+mEnd)/2; }
    }

    // this is a guide step that uses camera image data to
// guide the robot to the indicated bin of the cryptobox
//

    static public class driveUntilCryptoColumn extends Step{

        VuforiaLib_FTC2017 mVLib;
        OpMode mOpMode;
        int targetColumn;
        int currentColumn;

        String mVuMarkString;
        Pattern mPattern;
        ArrayList<ColumnHit> mPrevColumns;  // detected columns on previous pass
        ArrayList<ColumnHit> columnList;

        CameraLib.Filter mBlueFilter;

        SensorLib.PID mPid;
        float mPower;
        BNO055IMU mImu;
        boolean blue;
        DcMotor [] motors;

        public driveUntilCryptoColumn(OpMode op, VuforiaLib_FTC2017 VLib, String pattern, float power, int tColumn, boolean b, BNO055IMU imu, DcMotor [] m){
            mOpMode=op;
            mVLib=VLib;
            motors=m;
            blue=b;
            if(b){
                mPower=power;
            }else {
                mPower=-power;
            }
            targetColumn=tColumn;
            mPattern=Pattern.compile(pattern);
            final float Kp = 0.2f;         // degree heading proportional term correction per degree of deviation
            final float Ki = 0.0f;         // ... integrator term
            final float Kd = 0.0f;         // ... derivative term
            final float KiCutoff = 3.0f;   // maximum angle error for which we update integrator
            mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);
            mImu=imu;

        }

        public boolean loop() {
            super.loop();
            mVLib.loop();





            Bitmap bitmap = mVLib.getBitmap(8);                      // get uncropped, downsampled image from Vuforia
            CameraLib.CameraImage frame = new CameraLib.CameraImage(bitmap);       // .. and wrap it in a CameraImage

            if (bitmap != null && frame != null) {
                // look for cryptobox columns
                // get unfiltered view of colors (hues) by full-image-height column bands
                final int bandSize = 4;
                String colString = frame.columnHue(bandSize);

                //colString = new StringBuilder(colString).reverse().toString();//if the phone is upside down, the string needs to be reversed

                if (!blue) {
                    colString = new StringBuilder(colString).reverse().toString();//if we are on the red side we need to reverse the string because we are driving backwards
                }

                // log debug info ...
                mOpMode.telemetry.addData("hue columns", colString);

                // look for occurrences of given pattern of column colors
                ArrayList<ColumnHit> columns = new ArrayList<ColumnHit>(8);       // array of column start/end indices

                for (int i = 0; i < colString.length(); i++) {
                    // starting at position (i), look for the given pattern in the encoded (rgbcymw) scanline
                    Matcher m = mPattern.matcher(colString.substring(i));
                    if (m.lookingAt()) {
                        // add start/end info about this hit to the array
                        columns.add(new ColumnHit(i + m.start(), i + m.end() - 1));

                        // skip over this match
                        i += m.end();
                    }
                }

                // report the matches in telemetry
                for (ColumnHit h : columns) {
                    mOpMode.telemetry.addData("found ", "%s from %d to %d", mPattern.pattern(), h.start(), h.end());
                }
                if (columns != null && mPrevColumns != null) {

                    if (columns.size() > 0 && mPrevColumns.size() > 0) {
                        if (columns.get(0).start() == 0 && mPrevColumns.get(0).start() != 0) {
                            currentColumn++;
                        }
                    }
                }
                if (mPrevColumns == null) {
                    mPrevColumns = columns;
                }
                columnList = columns;


            }
            //if not driving straight, straighten out
            motors[0].setPower(motors[1].getPower() * (180 + mImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) / 180);
            motors[1].setPower(motors[1].getPower() * (180 + mImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) / 180);
            motors[2].setPower(motors[1].getPower() * (180 - mImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) / 180);
            motors[3].setPower(motors[1].getPower() * (180 - mImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) / 180);


            mOpMode.telemetry.addData("target", targetColumn);
            mOpMode.telemetry.addData("current", currentColumn);

            if (currentColumn == targetColumn) {
                motors[0].setPower(0);
                motors[1].setPower(0);
                motors[2].setPower(0);
                motors[3].setPower(0);
                return true;
            }


            mPrevColumns = columnList;

            mOpMode.telemetry.addData("target column", targetColumn);
            mOpMode.telemetry.addData("current column", currentColumn);
            return  false;
        }


    }
    static public class GoToCryptoBoxGuideStep extends AutoLib.MotorGuideStep implements SetMark {

        VuforiaLib_FTC2017 mVLib;
        String mVuMarkString;
        OpMode mOpMode;
        int mCBColumn;                      // which Cryptobox column we're looking for
        Pattern mPattern;                   // compiled regexp pattern we'll use to find the pattern we're looking for
        int mColumnOffset;                  // number of columns that have left the left-edge of the frame

        CameraLib.Filter mBlueFilter;       // filter to map cyan to blue

        ArrayList<ColumnHit> mPrevColumns;  // detected columns on previous pass

        SensorLib.PID mPid;                 // proportional–integral–derivative controller (PID controller)
        double mPrevTime;                   // time of previous loop() call
        ArrayList<AutoLib.SetPower> mMotorSteps;   // the motor steps we're guiding - assumed order is right ... left ...
        float mPower;                      // base power setting for motors
        int doneCount = 0;
        int numPrevColumn=0;

        public GoToCryptoBoxGuideStep(OpMode opMode, VuforiaLib_FTC2017 VLib, String pattern, float power) {
            mOpMode = opMode;
            mCBColumn = 1;     // if we never get a cryptobox directive from Vuforia, go for the first bin
            mPattern = Pattern.compile(pattern);    // look for the given pattern of column colors
            mBlueFilter = new BlueFilter();
            mVLib = VLib;
            mMotorSteps = null;     // this will be filled in by call from parent step
            mPower = power;
            mPrevColumns = null;
            mColumnOffset = 0;

            // construct a default PID controller for correcting heading errors
            final float Kp = 0.5f;         // degree heading proportional term correction per degree of deviation
            final float Ki = 0.0f;         // ... integrator term
            final float Kd = 0.0f;         // ... derivative term
            final float KiCutoff = 3.0f;   // maximum angle error for which we update integrator
            mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        }

        public void setMark(String s) {
            mVuMarkString = s;

            // compute index of column that forms the left side of the desired bin.
            // this assumes the camera is mounted to the left of the carried block.
            if (mVuMarkString == "LEFT")
                mCBColumn = 0;
            else
            if (mVuMarkString == "CENTER")
                mCBColumn = 1;
            else
            if (mVuMarkString == "RIGHT")
                mCBColumn = 2;

            // if the camera is on the right side of the block, we want the right edge of the bin.
            final boolean bCameraOnRight = false;
            if (bCameraOnRight)
                mCBColumn++;
        }

        public void set(ArrayList<AutoLib.SetPower> motorSteps){
            mMotorSteps = motorSteps;
        }

        public boolean loop() {
            super.loop();

            final int minDoneCount = 5;      // require "done" test to succeed this many consecutive times


            // initialize previous-time on our first call -> dt will be zero on first call
            if (firstLoopCall()) {
                mPrevTime = mOpMode.getRuntime();           // use timer provided by OpMode
            }

            mOpMode.telemetry.addData("VuMark", "%s found", mVuMarkString);

            // get most recent frame from camera (through Vuforia)
            //RectF rect = new RectF(0,0.25f,1f,0.75f);      // middle half of the image should be enough
            //Bitmap bitmap = mVLib.getBitmap(rect, 4);                      // get cropped, downsampled image from Vuforia
            Bitmap bitmap = mVLib.getBitmap(8);                      // get uncropped, downsampled image from Vuforia
            CameraLib.CameraImage frame = new CameraLib.CameraImage(bitmap);       // .. and wrap it in a CameraImage

            if (bitmap != null && frame != null) {
                // look for cryptobox columns
                // get unfiltered view of colors (hues) by full-image-height column bands
                final int bandSize = 4;
                String colString = frame.columnHue(bandSize);

                colString = new StringBuilder(colString).reverse().toString();//if the phone is upside down, the string needs to be reversed


                // log debug info ...
                mOpMode.telemetry.addData("hue columns", colString);

                // look for occurrences of given pattern of column colors
                ArrayList<ColumnHit> columns = new ArrayList<ColumnHit>(8);       // array of column start/end indices

                for (int i=0; i<colString.length(); i++) {
                    // starting at position (i), look for the given pattern in the encoded (rgbcymw) scanline
                    Matcher m = mPattern.matcher(colString.substring(i));
                    if (m.lookingAt()) {
                        // add start/end info about this hit to the array
                        columns.add(new ColumnHit(i+m.start(), i+m.end()-1));

                        // skip over this match
                        i += m.end();
                    }
                }

                // report the matches in telemetry
                for (ColumnHit h : columns) {
                    mOpMode.telemetry.addData("found ", "%s from %d to %d", mPattern.pattern(), h.start(), h.end());
                }

                int nCol = columns.size();

                // compute average distance between columns = distance between outermost / #bins
                float avgBinWidth = nCol>1 ? (float)(columns.get(nCol-1).end() - columns.get(0).start()) / (float)(nCol-1) : 0;

                // try to handle case where a column has left (or entered) the left-edge of the frame between the prev view and this one
                /*if (mPrevColumns != null  &&  mPrevColumns.size()>0  &&  nCol>0  && avgBinWidth > 0) {

                    // if the left-most column of the previous frame started at the left edge of the frame
                    // and the left edge of the current left-most column is about a bin-width to the right of the right edge of the
                    // left-most column of the previous frame
                    // then it's probably the case that the current left-most column is actually the second column of the previous frame.
                    if (mPrevColumns.get(0).start() == 0 && columns.get(0).start() > (mPrevColumns.get(0).end()+0.5*avgBinWidth)) {
                        mColumnOffset++;
                    }

                    // if the left-most column of the previous frame was not near the left edge of the frame
                    // but now there is a column at the left edge, then one probably entered the frame.
                    if (mColumnOffset > 0 && mPrevColumns.get(0).start() > 0.5*avgBinWidth  &&  columns.get(0).start() == 0) {
                        mColumnOffset--;
                    }
                }*/
                if(mPrevColumns !=null && mPrevColumns.size()>1 && nCol>1){
                    //if the leftmost column in the current frame is closer to the second column of the previous frame than to the leftmost
                    if(Math.abs(mPrevColumns.get(1).mid()-columns.get(0).mid())< Math.abs(mPrevColumns.get(0).mid()-columns.get(0).mid())){

                        if(mColumnOffset<4){
                            mColumnOffset++;
                        }
                    }
                    //if the leftmost column in the previous frame is closer to the second column of the previous frame than to the leftmost

                    else if(Math.abs(mPrevColumns.get(0).mid()-columns.get(1).mid())< Math.abs(mPrevColumns.get(0).mid()-columns.get(0).mid())){

                        if(mColumnOffset>0){
                            mColumnOffset--;
                        }
                    }
                }

                mOpMode.telemetry.addData("data", "avgWidth= %f  mColOff=%d", avgBinWidth, mColumnOffset);

                // if we found some columns, try to correct course using their positions in the image
                if (mCBColumn >= mColumnOffset && nCol > mCBColumn-mColumnOffset) {
                    // to start, we need to see all four columns to know where we're going ...
                    // after that, we try to match up the columns visible in this view with those from the previous pass
                    // TBD

                    // compute camera offset from near-side column of target bin (whichever side camera is to the block holder)
                    final float cameraOffset = -0.33f;        // e.g. camera is 0.2 x bin width to the right of block centerline
                    float cameraBinOffset = avgBinWidth * cameraOffset;
                    // camera target is center of target column + camera offset in image-string space
                    float cameraTarget = columns.get(mCBColumn-mColumnOffset).mid() + cameraBinOffset;

                    // the above computed target point should be in the middle of the image if we're on course -
                    // if not, correct our course to center it --
                    // compute fractional error = fraction of image offset of target from center = [-1 .. +1]
                    float error = (cameraTarget - (float)colString.length()/2.0f) / ((float)colString.length()/2.0f);

                    // compute motor correction from error through PID --
                    // for now, convert image-string error to angle and use standard "gyro" PID
                    final float cameraHalfFOVdeg = 28.0f;       // half angle FOV is about 28 degrees
                    float angError = error * cameraHalfFOVdeg;

                    mOpMode.telemetry.addData("data", "target=%f  error=%f angError=%f", cameraTarget, error, angError);

                    // compute delta time since last call -- used for integration time of PID step
                    double time = mOpMode.getRuntime();
                    double dt = time - mPrevTime;
                    mPrevTime = time;

                    // feed error through PID to get motor power correction value
                    float correction = -mPid.loop(error, (float)dt);

                    // compute new right/left motor powers
                    float rightPower = mPower + correction;
                    float leftPower = mPower - correction;

                    // normalize so neither has magnitude > 1
                    float norm = AutoLib.normalize(rightPower, leftPower);
                    rightPower *= norm;
                    leftPower *= norm;

                    // set the motor powers -- handle both time-based and encoder-based motor Steps
                    // assumed order is right motors followed by an equal number of left motors
                    int i = 0;
                    for (AutoLib.SetPower ms : mMotorSteps) {
                        ms.setPower((i++ < mMotorSteps.size()/2) ? rightPower : leftPower);
                    }

                    mOpMode.telemetry.addData("motors", "left=%f right=%f", leftPower, rightPower);
                }

              /*  if(nCol==0&&numPrevColumn!=0){
                    if (++doneCount >= minDoneCount) {
                        // stop all the motors and return "done"
                        for (AutoLib.SetPower ms : mMotorSteps) {
                            ms.setPower(0.0);
                        }
                        return true;
                    }

                }// when we're really close ... i.e. when the bin width is really big ... we're done*/
               // else if (nCol > 1 && avgBinWidth > colString.length()/2) {          // for now, when bin width > 1/2 FOV
                if (nCol > 1 && avgBinWidth > colString.length()/2) {
                    // require completion test to pass some min number of times in a row to believe it
                    if (++doneCount >= minDoneCount) {
                        // stop all the motors and return "done"
                        for (AutoLib.SetPower ms : mMotorSteps) {
                            ms.setPower(0.0);
                        }
                        return true;
                    }
                }
                else {
                    doneCount = 0;         // reset the "done" counter
                   /* if(mPrevColumns!=null) {
                        numPrevColumn = mPrevColumns.size();
                    }else{
                        numPrevColumn=0;
                    }*/
                }
                mOpMode.telemetry.addData("data", "doneCount=%d", doneCount);

                // save column hits for next pass to help handle columns leaving the field of view of
                // the camera as we get close.
                mPrevColumns = columns;

            }

            return false;  // haven't found anything yet
        }

        public void stop() {
        }
    }





    static public class GoToCryptoBoxColumn extends Step{

        VuforiaLib_FTC2017 mVLib;
        String mVuMarkString;
        OpMode mOpMode;
        int mCBColumn;                      // which Cryptobox column we're looking for
        Pattern mPattern;                   // compiled regexp pattern we'll use to find the pattern we're looking for
        int mColumnOffset;                  // number of columns that have left the left-edge of the frame
        DcMotor [] motors;

        CameraLib.Filter mBlueFilter;       // filter to map cyan to blue

        ArrayList<ColumnHit> mPrevColumns;  // detected columns on previous pass

        SensorLib.PID mPid;                 // proportional–integral–derivative controller (PID controller)
        double mPrevTime;                   // time of previous loop() call
        //ArrayList<AutoLib.SetPower> mMotorSteps;   // the motor steps we're guiding - assumed order is right ... left ...
        float mPower;                      // base power setting for motors
        int doneCount = 0;
        int numPrevColumn=0;

        public GoToCryptoBoxColumn(OpMode opMode, VuforiaLib_FTC2017 VLib, String pattern, DcMotor [] m, float power) {
            mOpMode = opMode;
            mCBColumn = 1;     // if we never get a cryptobox directive from Vuforia, go for the first bin
            mPattern = Pattern.compile(pattern);    // look for the given pattern of column colors
            mBlueFilter = new BlueFilter();
            mVLib = VLib;
            motors=m;
            if(pattern=="^r+"){
                mPower = -power;
            }
            else{
                mPower = power;
            }

            mPrevColumns = null;
            mColumnOffset = 0;

            // construct a default PID controller for correcting heading errors
            final float Kp = 0.5f;         // degree heading proportional term correction per degree of deviation
            final float Ki = 0.0f;         // ... integrator term
            final float Kd = 0.0f;         // ... derivative term
            final float KiCutoff = 3.0f;   // maximum angle error for which we update integrator
            mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        }

        public void setMark(String s) {
            mVuMarkString = s;

            // compute index of column that forms the left side of the desired bin.
            // this assumes the camera is mounted to the left of the carried block.
            if (mVuMarkString == "LEFT")
                mCBColumn = 0;
            else
            if (mVuMarkString == "CENTER")
                mCBColumn = 1;
            else
            if (mVuMarkString == "RIGHT")
                mCBColumn = 2;

            // if the camera is on the right side of the block, we want the right edge of the bin.
            final boolean bCameraOnRight = false;
            if (bCameraOnRight)
                mCBColumn++;
        }

       // public void set(ArrayList<AutoLib.SetPower> motorSteps){
       //     mMotorSteps = motorSteps;
       // }

        public boolean loop() {
            super.loop();

            final int minDoneCount = 5;      // require "done" test to succeed this many consecutive times


            // initialize previous-time on our first call -> dt will be zero on first call
            if (firstLoopCall()) {
                setMark(mVLib.toString());
                mPrevTime = mOpMode.getRuntime();           // use timer provided by OpMode
                motors[0].setPower(mPower);
                motors[1].setPower(mPower);
                motors[2].setPower(mPower);
                motors[3].setPower(mPower);


            }

            mOpMode.telemetry.addData("VuMark", "%s found", mVuMarkString);

            // get most recent frame from camera (through Vuforia)
            //RectF rect = new RectF(0,0.25f,1f,0.75f);      // middle half of the image should be enough
            //Bitmap bitmap = mVLib.getBitmap(rect, 4);                      // get cropped, downsampled image from Vuforia
            Bitmap bitmap = mVLib.getBitmap(8);                      // get uncropped, downsampled image from Vuforia
            CameraLib.CameraImage frame = new CameraLib.CameraImage(bitmap);       // .. and wrap it in a CameraImage

            if (bitmap != null && frame != null) {
                // look for cryptobox columns
                // get unfiltered view of colors (hues) by full-image-height column bands
                final int bandSize = 4;
                String colString = frame.columnHue(bandSize);

               // colString = new StringBuilder(colString).reverse().toString();//if the phone is upside down, the string needs to be reversed


                // log debug info ...
                mOpMode.telemetry.addData("hue columns", colString);

                // look for occurrences of given pattern of column colors
                ArrayList<ColumnHit> columns = new ArrayList<ColumnHit>(8);       // array of column start/end indices

                for (int i=0; i<colString.length(); i++) {
                    // starting at position (i), look for the given pattern in the encoded (rgbcymw) scanline
                    Matcher m = mPattern.matcher(colString.substring(i));
                    if (m.lookingAt()) {
                        // add start/end info about this hit to the array
                        columns.add(new ColumnHit(i+m.start(), i+m.end()-1));

                        // skip over this match
                        i += m.end();
                    }
                }

                // report the matches in telemetry
                for (ColumnHit h : columns) {
                    mOpMode.telemetry.addData("found ","%s from %d to %d", mPattern.pattern(), h.start(), h.end());
                }

                int nCol = columns.size();

                // compute average distance between columns = distance between outermost / #bins
                float avgBinWidth = nCol>1 ? (float)(columns.get(nCol-1).end() - columns.get(0).start()) / (float)(nCol-1) : 0;




                mOpMode.telemetry.addData("data", "avgWidth= %f  mColOff=%d", avgBinWidth, mColumnOffset);



                mOpMode.telemetry.addData("data", "doneCount=%d", doneCount);

                // save column hits for next pass to help handle columns leaving the field of view of
                // the camera as we get close.
                mPrevColumns = columns;

            }

            return false;  // haven't found anything yet
        }

        public void stop() {
        }
    }






















    // a Step that drives a Servo to a given position
    static public class ServoStep extends Step {
        Servo mServo;
        double mPosition;          // target position of servo
        double mTolerance;

        public ServoStep(Servo servo, double position) {
            mServo = servo;
            mPosition = position;
            mTolerance = 0.01;      // 1% of default 0..1 range
        }

        // if servo is set to non-default range, set tolerance here
        public void setTolerance(double tolerance) {
            mTolerance = tolerance;
        }

        public boolean loop() {
            super.loop();

            // tell the servo to go to the target position on the first call
            if (firstLoopCall()) {
                mServo.setPosition(mPosition);
            }

            // we're done when the servo gets to the ordered position (within tolerance)
            boolean done = Math.abs(mPosition-mServo.getPosition()) < mTolerance;

            return done;
        }

    }



    // a Step that provides Vuforia-based guidance to motors controlled by other concurrent Steps (e.g. encoder or time-based)
    // assumes an even number of concurrent drive motor steps assuming order fr, br, fl, bl driving Squirrely Wheels
    // (i.e. wheels that can move the robot in any direction without yawing the robot itself).
    // this step tries to keep the robot on course to a given location on the field.
    static public class VuforiaSquirrelyGuideStep extends Step {
        private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
        private OpMode mOpMode;                             // needed so we can log output (may be null)
        private ArrayList<SetPower> mMotorSteps;            // the motor steps we're guiding - assumed order is right ... left ...
        private VectorF mTargetPosition;                    // target position on field
        private LocationSensor mLocSensor;                  // sensor to use for field location information (e.g. Vuforia)
        private HeadingSensor mYawSensor;                   // sensor to use for robot orientation on field (may be same as LocationSensor)
        private float mError;                               // how close do we have to be to declare "done"
        private boolean mStop;                              // stop motors when target position is reached

        public VuforiaSquirrelyGuideStep(OpMode mode, VectorF targetPosition, LocationSensor locSensor, HeadingSensor yawSensor,
                                         ArrayList<SetPower> motorSteps, float power, float error, boolean stop)
        {
            mOpMode = mode;
            mTargetPosition = targetPosition;
            mLocSensor = locSensor;
            mYawSensor = yawSensor;
            mMotorSteps = motorSteps;
            mPower = power;
            mError = error;
            mStop = stop;
        }

        private void doSearch()
        {
            // turn slowly CCW searching for a target
            final double searchPower = 0.2;
            mMotorSteps.get(0).setPower(searchPower);
            mMotorSteps.get(1).setPower(searchPower);
            mMotorSteps.get(2).setPower(-searchPower);
            mMotorSteps.get(3).setPower(-searchPower);
        }

        public boolean loop()
        {
            // check for valid location data - if not, do simple search pattern to try to acquire data
            if (!mLocSensor.haveLocation()) {
                doSearch();
                return false;       // not done
            }

            // compute absolute direction vector to target position on field
            VectorF position = mLocSensor.getLocation();
            VectorF dirToTarget = mTargetPosition.subtracted(position);
            dirToTarget.put(2, 0.0f);        // ignore Z when computing distance to target

            // compute absolute field heading to target: zero aligned with Y axis, positive CCW, degrees 0/359
            double headingToTarget = Math.atan2(-dirToTarget.get(0), dirToTarget.get(1));
            headingToTarget *= 180.0/Math.PI;       // to degrees

            // check for valid heading data - if not, do simple search pattern to try to acquire data
            if (!mYawSensor.haveHeading()) {
                doSearch();
                return false;
            }

            // get current orientation of the robot on the field
            double robotYaw = mYawSensor.getHeading();

            // compute relative heading robot should move along
            double robotHeading = headingToTarget - robotYaw;

            // compute motor powers needed to go in that direction
            MotorPowers mp = GetSquirrelyWheelMotorPowers(robotHeading);
            double frontPower = mp.Front() * mPower;
            double backPower = mp.Back() * mPower;

            // reduce motor powers when we're very close to the target position
            final double slowDist = 3.0*mError;   // start slowing down when we're this close
            double distToTarget = dirToTarget.magnitude();
            if (distToTarget < slowDist) {
                frontPower *= distToTarget/slowDist;
                backPower  *= distToTarget/slowDist;
            }

            // are we there yet?
            boolean bDone = distToTarget < mError;     // within an inch of target position?

            // optionally stop motors when we reach the target position
            if (bDone && mStop)
                frontPower = backPower = 0;

            // output debug telemetry
            mOpMode.telemetry.addData("VSGS:", "target position: %s", mTargetPosition.multiplied(1.0f/25.4f).toString());    // inches
            mOpMode.telemetry.addData("VSGS:", "abs heading: %4.1f  distance: %4.1f", headingToTarget, distToTarget/25.4);   // degrees, inches

            // update motors
            // assumed order is fr, br, fl, bl
            mMotorSteps.get(0).setPower(frontPower);
            mMotorSteps.get(1).setPower(backPower);
            mMotorSteps.get(2).setPower(frontPower);
            mMotorSteps.get(3).setPower(backPower);

            return bDone;
        }
    }

    // a Step that uses Vuforia input to drive a SquirrelyWheel robot to a given position on the field.
    // uses a VuforiaSquirrelyGuideStep to adjust power to the 4 motors, assuming order fr, br, fl, bl.
    static public class VuforiaSquirrelyDriveStep extends ConcurrentSequence {

        public VuforiaSquirrelyDriveStep(OpMode mode, VectorF targetPosition, LocationSensor locSensor, HeadingSensor yawSensor,
                                         DcMotor motors[], float power, float error, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, 0, false);  // always set requested power and return "done"
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on Vuforia input -
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new VuforiaSquirrelyGuideStep(mode, targetPosition, locSensor, yawSensor, steps, power, error, stop));

        }

        // the base class loop function does all we need --
        // since the motors always return done, the composite step will return "done" when
        // the GuideStep says it's done, i.e. we've reached the target location.

    }

    // a Step that provides Vuforia-based guidance to motors controlled by other concurrent Steps (e.g. encoder or time-based)
    // assumes an even number of concurrent drive motor steps assuming order fr, br, fl, bl driving normal wheels
    // this step tries to keep the robot on course to a given location on the field.
    static public class VuforiaGuideStep extends Step {
        private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
        private OpMode mOpMode;                             // needed so we can log output (may be null)
        private ArrayList<SetPower> mMotorSteps;            // the motor steps we're guiding - assumed order is right ... left ...
        private VectorF mTargetPosition;                    // target position on field
        private LocationSensor mLocSensor;                  // sensor to use for field location information (e.g. Vuforia)
        private HeadingSensor mYawSensor;                   // sensor to use for robot orientation on field (may be same as LocationSensor)
        private float mError;                               // how close do we have to be to declare "done"
        private SensorLib.PID mPid;                         // PID controller to compute motor corrections from Vuforia heading data
        private double mPrevTime;                           // time of previous loop() call
        private boolean mStop;                              // stop motors when target position is reached

        public VuforiaGuideStep(OpMode mode, VectorF targetPosition, LocationSensor locSensor, HeadingSensor yawSensor,
                                         ArrayList<SetPower> motorSteps, float power, float error, boolean stop)
        {
            mOpMode = mode;
            mTargetPosition = targetPosition;
            mLocSensor = locSensor;
            mYawSensor = yawSensor;
            mMotorSteps = motorSteps;
            mPower = power;
            mError = error;
            mStop = stop;

            // parameters of the PID controller for this sequence
            float Kp = 0.035f;        // motor power proportional term correction per degree of deviation
            float Ki = 0.02f;         // ... integrator term
            float Kd = 0;             // ... derivative term
            float KiCutoff = 3.0f;    // maximum angle error for which we update integrator

            // create a PID controller for the sequence
            mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);    // make the object that implements PID control algorithm
        }

        public boolean loop()
        {
            // initialize previous-time on our first call -> dt will be zero on first call
            if (firstLoopCall()) {
                mPrevTime = mOpMode.getRuntime();           // use timer provided by OpMode
            }

            float rightPower = 0;
            float leftPower = 0;
            boolean bDone = false;

            // check for valid location and heading input --
            // if either is absent, do a slow circling maneuver to try to acquire
            if (!mLocSensor.haveLocation() || !mYawSensor.haveHeading()) {
                rightPower = 0.2f;
                leftPower  = -rightPower;
            }

            else {
                // compute absolute direction vector to target position on field
                VectorF position = mLocSensor.getLocation();
                if (position == null)
                    return false;       // we have an inconsistency problem but we're not done

                VectorF dirToTarget = mTargetPosition.subtracted(position);
                dirToTarget.put(2, 0.0f);        // ignore Z when computing distance to target

                // compute absolute field heading to target: zero aligned with Y axis, positive CCW, degrees 0/359
                float headingToTarget = (float) (Math.atan2(-dirToTarget.get(0), dirToTarget.get(1)));
                headingToTarget *= 180.0 / Math.PI;       // to degrees

                // try to keep the robot facing toward the corner between the sides of the fields with Vuforia targets --
                // if direction to target is away from that corner of the field, run backwards to the target; otherwise forwards.
                // i.e. if directionToTarget dot (-1,-1) < 0, then we're heading away from the "good" corner
                boolean destAwayFromTargets = dirToTarget.dotProduct(new VectorF(-1, -1, 0)) < 0;

                float robotHeading = mYawSensor.getHeading();     // get latest reading from direction sensor
                // convention is positive angles CCW, wrapping from 359-0

                // if the target position is behind us, reverse the robot orientation direction so
                // we can compute correction to go in the desired direction toward the target (backwards)
                if (destAwayFromTargets) {
                    robotHeading = SensorLib.Utils.wrapAngle(robotHeading + 180);
                }

                float error = SensorLib.Utils.wrapAngle(robotHeading - headingToTarget);   // deviation from desired heading
                // deviations to left are positive, to right are negative

                // compute delta time since last call -- used for integration time of PID step
                double time = mOpMode.getRuntime();
                double dt = time - mPrevTime;
                mPrevTime = time;

                // feed error through PID to get motor power correction value
                float correction = -mPid.loop(error, (float) dt);

                // compute new right/left motor powers
                float powerDir = (destAwayFromTargets) ? -mPower : mPower;
                rightPower = Range.clip(powerDir + correction, -1, 1);
                leftPower = Range.clip(powerDir - correction, -1, 1);

                // reduce motor powers when we're very close to the target position
                final double slowDist = 3.0*mError;   // start slowing down when we're this close
                double distToTarget = dirToTarget.magnitude();
                if (distToTarget < slowDist) {
                    leftPower *= distToTarget / slowDist;
                    rightPower *= distToTarget / slowDist;
                }

                // are we there yet?
                bDone = distToTarget < mError;     // within given distance of target position?

                // optionally stop motors when we reach the target position
                if (bDone && mStop)
                    leftPower = rightPower = 0;

                // output debug telemetry
                mOpMode.telemetry.addData("VGS:", "abs heading: %4.1f  distance: %4.1f", headingToTarget, distToTarget/25.4);   // degrees, inches
            }

            // set the motor powers -- handle both time-based and encoder-based motor Steps
            // assumed order is right motors followed by an equal number of left motors
            int i = 0;
            for (SetPower ms : mMotorSteps) {
                ms.setPower((i++ < mMotorSteps.size()/2) ? rightPower : leftPower);
            }

            // output debug telemetry
            mOpMode.telemetry.addData("VGS:", "target position: %s", mTargetPosition.multiplied(1.0f/25.4f).toString());    // inches

            return bDone;
        }
    }

    // a Step that uses Vuforia input to drive a normal wheel robot to a given position on the field.
    // uses a VuforiaGuideStep to adjust power to the 4 motors, assuming order fr, br, fl, bl.
    static public class VuforiaDriveStep extends ConcurrentSequence {

        public VuforiaDriveStep(OpMode mode, VectorF targetPosition, LocationSensor locSensor, HeadingSensor yawSensor,
                                         DcMotor motors[], float power, float error, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, 0, false);  // always set requested power and return "done"
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on Vuforia input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new VuforiaGuideStep(mode, targetPosition, locSensor, yawSensor, steps, power, error, stop));

        }

        // the base class loop function does all we need --
        // since the motors always return done, the composite step will return "done" when
        // the GuideStep says it's done, i.e. we've reached the target location.

    }

    // a Step that waits for valid location and heading data to be available --- e.g from Vuforia --
    // when added to either a dead reckoning or gyro-based movement step, it can be used to end that step
    // when we're close enough to the targets for Vuforia to start being used. The base step should be
    // of "zero-length" -- i.e. it should always be "done" so the composite step will be "done" as soon as
    // this step detects valid location and heading data.
    static public class LocationHeadingWaitStep extends Step {

        private LocationSensor mLocSensor;                  // sensor to use for field location information (e.g. Vuforia)
        private HeadingSensor mYawSensor;                   // sensor to use for robot orientation on field (may be same as LocationSensor)

        public LocationHeadingWaitStep(LocationSensor locSensor, HeadingSensor yawSensor)
        {
            mLocSensor = locSensor;
            mYawSensor = yawSensor;
        }

        public boolean loop()
        {
            boolean bDone = true;
            if (mLocSensor != null)
                bDone &= mLocSensor.haveLocation();
            if (mYawSensor != null)
                bDone &= mYawSensor.haveHeading();
            return bDone;
        }
    }


    // a Step that provides gyro-based guidance to motors controlled by other concurrent Steps (e.g. encoder or time-based)
    // assumes an even number of concurrent drive motor steps in order right ..., left ...
    // this step tries to keep the robot on the given course by adjusting the left vs. right motors to change the robot's heading.
    static public class GyroGuideStep extends Step {
        private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
        private float mHeading;                             // compass heading to steer for (-180 .. +180 degrees)
        private OpMode mOpMode;                             // needed so we can log output (may be null)
        private HeadingSensor mGyro;                        // sensor to use for heading information (e.g. Gyro or Vuforia)
        private SensorLib.PID mPid;                         // proportional–integral–derivative controller (PID controller)
        private double mPrevTime;                           // time of previous loop() call
        private ArrayList<SetPower> mMotorSteps;            // the motor steps we're guiding - assumed order is right ... left ...

        public GyroGuideStep(OpMode mode, float heading, HeadingSensor gyro, SensorLib.PID pid,
                             ArrayList<SetPower> motorsteps, float power)
        {
            mOpMode = mode;
            mHeading = heading;
            mGyro = gyro;
            mPid = pid;
            mMotorSteps = motorsteps;
            mPower = power;
        }

        public boolean loop()
        {
            // initialize previous-time on our first call -> dt will be zero on first call
            if (firstLoopCall()) {
                mPrevTime = mOpMode.getRuntime();           // use timer provided by OpMode
            }

            float heading = mGyro.getHeading();     // get latest reading from direction sensor
            // convention is positive angles CCW, wrapping from 359-0

            float error = SensorLib.Utils.wrapAngle(heading-mHeading);   // deviation from desired heading
            // deviations to left are positive, to right are negative

            // compute delta time since last call -- used for integration time of PID step
            double time = mOpMode.getRuntime();
            double dt = time - mPrevTime;
            mPrevTime = time;

            // feed error through PID to get motor power correction value
            float correction = -mPid.loop(error, (float)dt);

            // compute new right/left motor powers
            float rightPower = Range.clip(mPower + correction, -1, 1);
            float leftPower = Range.clip(mPower - correction, -1, 1);

            // set the motor powers -- handle both time-based and encoder-based motor Steps
            // assumed order is right motors followed by an equal number of left motors
            int i = 0;
            for (SetPower ms : mMotorSteps) {
                ms.setPower((i++ < mMotorSteps.size()/2) ? rightPower : leftPower);
            }

            // log some data
            if (mOpMode != null) {
                mOpMode.telemetry.addData("heading ", heading);
                mOpMode.telemetry.addData("left power ", leftPower);
                mOpMode.telemetry.addData("right power ", rightPower);
            }

            // guidance step always returns "done" so the CS in which it is embedded completes when
            // all the motors it's controlling are done
            return true;
        }
    }

    // a Step that uses gyro input to drive along a given course for a given distance given by motor encoders.
    // uses a GyroGuideStep to adjust power to 2 or 4 motors.
    // assumes a robot with up to 4 drive motors in assumed order right motors, left motors
    static public class AzimuthTimedDriveStep extends ConcurrentSequence {

        public AzimuthTimedDriveStep(OpMode mode, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                     DcMotor motors[], float power, float time, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, time, stop);
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new GyroGuideStep(mode, heading, gyro, pid, steps, power));

        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

    }

    // a Step that uses gyro input to drive along a given course for a given distance given by motor encoders.
    // uses a GyroGuideStep to adjust power to 2 or 4 motors.
    // assumes a robot with up to 4 drive motors in assumed order right motors, left motors
    static public class AzimuthCountedDriveStep extends ConcurrentSequence {

        public AzimuthCountedDriveStep(OpMode mode, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                       DcMotor motors[], float power, int count, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    EncoderMotorStep step = new EncoderMotorStep(em, power, count, stop);
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new GyroGuideStep(mode, heading, gyro, pid, steps, power));

        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

    }


    // some convenience utility classes for common operations

    // a Sequence that moves an up-to-four-motor robot in a straight line with given power for given time
    static public class MoveByTimeStep extends ConcurrentSequence {

        public MoveByTimeStep(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double power, double seconds, boolean stop)
        {
            if (fr != null)
                this.add(new TimedMotorStep(fr, power, seconds, stop));
            if (br != null)
                this.add(new TimedMotorStep(br, power, seconds, stop));
            if (fl != null)
                this.add(new TimedMotorStep(fl, power, seconds, stop));
            if (bl != null)
                this.add(new TimedMotorStep(bl, power, seconds, stop));
        }

        public MoveByTimeStep(DcMotor motors[], double power, double seconds, boolean stop)
        {
            for (DcMotor em : motors)
                if (em != null)
                    this.add(new TimedMotorStep(em, power, seconds, stop));
        }

    }


    // a Sequence that turns an up-to-four-motor robot by applying the given right and left powers for given time
    static public class TurnByTimeStep extends ConcurrentSequence {

        public TurnByTimeStep(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double rightPower, double leftPower, double seconds, boolean stop)
        {
            if (fr != null)
                this.add(new TimedMotorStep(fr, rightPower, seconds, stop));
            if (br != null)
                this.add(new TimedMotorStep(br, rightPower, seconds, stop));
            if (fl != null)
                this.add(new TimedMotorStep(fl, leftPower, seconds, stop));
            if (bl != null)
                this.add(new TimedMotorStep(bl, leftPower, seconds, stop));
        }

    }


    // a Sequence that moves an up-to-four-motor robot in a straight line with given power for given encoder count
    static public class MoveByEncoderStep extends ConcurrentSequence {

        public MoveByEncoderStep(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double power, int count, boolean stop)
        {
            if (fr != null)
                this.add(new EncoderMotorStep(fr, power, count, stop));
            if (br != null)
                this.add(new EncoderMotorStep(br, power, count, stop));
            if (fl != null)
                this.add(new EncoderMotorStep(fl, power, count, stop));
            if (bl != null)
                this.add(new EncoderMotorStep(bl, power, count, stop));
        }

        public MoveByEncoderStep(DcMotor motors[], double power, int count, boolean stop)
        {
            for (DcMotor em : motors)
                if (em != null)
                    this.add(new EncoderMotorStep(em, power, count, stop));
        }

    }


    // a Sequence that turns an up-to-four-motor robot by applying the given right and left powers for given right and left encoder counts
    static public class TurnByEncoderStep extends ConcurrentSequence {

        public TurnByEncoderStep(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double rightPower, double leftPower, int rightCount, int leftCount, boolean stop)
        {
            if (fr != null)
                this.add(new EncoderMotorStep(fr, rightPower, rightCount, stop));
            if (br != null)
                this.add(new EncoderMotorStep(br, rightPower, rightCount, stop));
            if (fl != null)
                this.add(new EncoderMotorStep(fl, leftPower, leftCount, stop));
            if (bl != null)
                this.add(new EncoderMotorStep(bl, leftPower, leftCount, stop));
        }

    }

    /// some utility functions

    // linear interpolation
    private static double lerp (double x, double x0, double x1, double y0, double y1)
    {
        return ((x-x0)/(x1-x0))*(y1-y0) + y0;
    }

    // return normalization factor that makes max magnitude of any argument 1
    static public float normalize(float a, float b)
    {
        float m = Math.max(Math.abs(a), Math.abs(b));
        return (m > 1.0f) ? 1.0f/m : 1.0f;
    }

    static public float normalize(float a, float b, float c, float d)
    {
        float m = Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.max(Math.abs(c), Math.abs(d)));
        return (m > 1.0f) ? 1.0f/m : 1.0f;
    }

    // return normalization factor that makes max magnitude of any argument 1
    static public double normalize(double a, double b)
    {
        double m = Math.max(Math.abs(a), Math.abs(b));
        return (m > 1.0) ? 1.0/m : 1.0;
    }

    static public double normalize(double a, double b, double c, double d)
    {
        double m = Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.max(Math.abs(c), Math.abs(d)));
        return (m > 1.0) ? 1.0/m : 1.0;
    }

    static public float normalize(float[] a)
    {
        float m = 0;
        for (float x : a)
            m = Math.max(m, Math.abs(x));
        return (m > 1.0f) ? 1.0f/m : 1.0f;
    }

    static public double normalize(double[] a)
    {
        double m = 0;
        for (double x : a)
            m = Math.max(m, Math.abs(x));
        return (m > 1.0) ? 1.0/m : 1.0;
    }
    // some Steps that use various sensor input to control other Steps

    // interface for setting the current power of either kind of MotorStep
    public interface SetMotorSteps {
        public void set(ArrayList<AutoLib.SetPower> motorsteps);
    }

    static public abstract class MotorGuideStep extends AutoLib.Step implements SetMotorSteps {
        public void set(ArrayList<AutoLib.SetPower> motorsteps){}
    }


    // a Step that provides gyro-based guidance to motors controlled by other concurrent Steps (e.g. encoder or time-based)
    // assumes an even number of concurrent drive motor steps in order right ..., left ...
    // this step tries to keep the robot on the given course by adjusting the left vs. right motors to change the robot's heading.


    // a Step that waits for valid location and heading data to be available --- e.g from Vuforia --
    // when added to either a dead reckoning or gyro-based movement step, it can be used to end that step
    // when we're close enough to the targets for Vuforia to start being used. The base step should be
    // of "zero-length" -- i.e. it should always be "done" so the composite step will be "done" as soon as
    // this step detects valid location and heading data.


    // a Step that returns true iff the given HeadingSensor reports a heading within some tolerance of a desired heading.
    static public class GyroTestHeadingStep extends Step {
        private HeadingSensor mSensor;
        private double mHeading;
        private double mTolerance;

        public GyroTestHeadingStep(HeadingSensor sensor, double heading, double tol){
            mSensor = sensor;
            mHeading = heading;
            mTolerance = tol;
        }

        public boolean loop() {
            super.loop();

            if (mSensor.haveHeading())
                return (Math.abs(mSensor.getHeading()-mHeading) < mTolerance);
            else
                return false;
        }
    }

    // a Step that stops the given set of motor control steps and terminates
    // when the given DistanceSensor reports less than a given distance (in mm).
    // pass in an empty set of motors to just do the test (e.g. for use with WaitTimeTestStep).
   /*static public class DistanceSensorGuideStep extends Step {

        private OpMode mOpMode;                     // for telemetry output (optional)
        private DistanceSensor mDistanceSensor;     // distance sensor to read
        private float mDistance;                    // stopping distance
        private ArrayList<SetPower> mSteps;         // motors to stop at given distance

        public DistanceSensorGuideStep(OpMode opmode, DistanceSensor ds, float distance, ArrayList<SetPower> steps)
        {
            mOpMode = opmode;
            mDistanceSensor = ds;
            mDistance = distance;
            mSteps = steps;
        }

        public boolean loop()
        {
            super.loop();

            boolean have = mDistanceSensor.haveDistance();
            float dist = mDistanceSensor.getDistance();
            boolean done = have && (dist < mDistance);
            if (done)
                for (SetPower step : mSteps)
                    step.setPower(0);
            if (mOpMode != null) {
                mOpMode.telemetry.addData("DSGS: ", "have = %b  dist(mm) = %2.1f  (in) = %2.1f  done = %b", have, dist, dist/25.4f, done);
            }
            return done;
        }
    }*/

    // a generic Step that uses a MotorGuideStep to steer the robot while driving along a given path
    // until the terminatorStep tells us that we're there, thereby terminating this step.
    static public class GuidedTerminatedDriveStep extends AutoLib.ConcurrentSequence {

        public GuidedTerminatedDriveStep(OpMode mode, AutoLib.MotorGuideStep guideStep, AutoLib.MotorGuideStep terminatorStep, DcMotor[] motors)
        {
            // add a concurrent Step to control each motor
            ArrayList<AutoLib.SetPower> steps = new ArrayList<AutoLib.SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    AutoLib.TimedMotorStep step = new AutoLib.TimedMotorStep(em, 0, 0, false);
                    // the guide or terminator Step will stop the motors and complete the sequence
                    this.add(step);
                    steps.add(step);
                }

            // if there's a separate terminator step, tell it about the motor steps and add it to the sequence
            if (terminatorStep != null) {
                terminatorStep.set(steps);
                this.preAdd(terminatorStep);
            }

            // tell the guideStep about the motor Steps it should control
            guideStep.set(steps);

            // add a concurrent Step to control the motor steps based on sensor (gyro, camera, etc.) input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            // and BEFORE the terminatorStep might try to turn the motors off.
            this.preAdd(guideStep);
        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

    }



    // some utilities to support "Squirrely Wheels" that move the robot sideways
    // when front and back wheels turn in opposite directions

    // example of a class used to return multiple values from a function call
    public static class MotorPowers {
        public double mFront;
        public double mBack;
        public MotorPowers(double front, double back) {
            mFront = front;
            mBack = back;
        }
        public double Front() { return mFront; }
        public double Back() { return mBack; }
    }

    // this function computes the relative front/back power settings needed to move along a given
    // heading, relative to the current orientation of the robot.
    public static MotorPowers GetSquirrelyWheelMotorPowers(
            double heading    // in degrees, zero = straight ahead, positive CCW, range +-180
    )
    {
        // wrap heading around to acceptable range
        heading = SensorLib.Utils.wrapAngle(heading);

        // compute front and back wheel relative speeds needed to go in desired direction
        double front = 0.0f;
        double back = 0.0f;
        if (heading < 0) {
            if (heading > -90) {
                front = 1.0;
                back = lerp(heading, 0, -90, 1, -1);
            } else {
                front = lerp(heading, -90, -180, 1, -1);
                back = -1.0;
            }
        }
        else {
            if (heading < 90) {
                front = lerp(heading, 0, 90, 1, -1);
                back = 1.0;
            }
            else {
                front = -1.0;
                back = lerp(heading, 90, 180, 1, -1);
            }
        }

        // return results
        return new MotorPowers(front, back);
    }

    // a Step that drives 4 "squirrely wheels" to move the robot in a given direction
    // relative to where it's facing, for a given time.
    static public class MoveSquirrelyByTimeStep extends ConcurrentSequence {

        public MoveSquirrelyByTimeStep(
                DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl,     // motors
                double heading,    // in degrees, zero = straight ahead, positive CCW, range +-180
                double power,      // overall power scaling factor
                double seconds,    // time in seconds
                boolean stop)
        {
            _MoveSquirrelyByTimeStep(fr, br, fl, bl, heading, power, seconds, stop);
        }

        public MoveSquirrelyByTimeStep(
                DcMotor motors[],  // motors -- assumed order is fr, br, fl, bl
                double heading,    // in degrees, zero = straight ahead, positive CCW, range +-180
                double power,      // overall power scaling factor
                double seconds,    // time in seconds
                boolean stop)
        {
            _MoveSquirrelyByTimeStep(motors[0], motors[1], motors[2], motors[3], heading, power, seconds, stop);
        }


        // internal function that actually does the constructor stuff for the two different ctors
        protected void _MoveSquirrelyByTimeStep(
                DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl,     // motors
                double heading,    // in degrees, zero = straight ahead, positive CCW, range +-180
                double power,      // overall power scaling factor
                double seconds,    // time in seconds
                boolean stop)
        {
            // compute relative front and back motor powers needed to move on the desired heading
            MotorPowers mp = GetSquirrelyWheelMotorPowers(heading);

            // create TimedMotorSteps to control the 4 motors
            if (fr != null)
                this.add(new TimedMotorStep(fr, mp.Front()*power, seconds, stop));
            if (br != null)
                this.add(new TimedMotorStep(br, mp.Back()*power, seconds, stop));
            if (fl != null)
                this.add(new TimedMotorStep(fl, mp.Front()*power, seconds, stop));
            if (bl != null)
                this.add(new TimedMotorStep(bl, mp.Back()*power, seconds, stop));
        }
    }


    // timer
    static public class Timer {
        long mStartTime;
        double mSeconds;

        public Timer(double seconds) {
            mStartTime = 0L;        // creation time is NOT start time
            mSeconds = seconds;
        }

        public void start() {
            mStartTime = System.nanoTime();
        }

        // return elapsed time in seconds since timer was created or restarted
        public double elapsed() {
            return (double) (System.nanoTime() - mStartTime) / (double) TimeUnit.SECONDS.toNanos(1L);
        }

        public double remaining() {
            return mSeconds - elapsed();
        }

        public boolean done() {
            return (remaining() <= 0);
        }
    }

    // test hardware classes -- useful for testing when no hardware is available.
    // these are primarily intended for use in testing autonomous mode code, but could
    // also be useful for testing tele-operator modes.

    // a dummy DcMotor that just logs commands we send to it --
    // useful for testing Motor code when you don't have real hardware handy
   /*static public class TestMotor implements DcMotor {
        OpMode mOpMode;     // needed for logging data
        String mName;       // string id of this motor
        double mPower;      // current power setting
        DcMotor.RunMode mMode;
        int mTargetPosition;
        int mCurrentPosition;
        boolean mPowerFloat;
        Direction mDirection;
        ZeroPowerBehavior mZeroPowerBehavior;
        int mMaxSpeed;

        public TestMotor(String name, OpMode opMode) {
            super();     // init base class (real DcMotor) with dummy data
            mOpMode = opMode;
            mName = name;
            mPower = 0.0;
            mMaxSpeed = 0;
            mMode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS;
            mTargetPosition = 0;
            mCurrentPosition = 0;
            mPowerFloat = false;
            mDirection = Direction.FORWARD;
            mZeroPowerBehavior = ZeroPowerBehavior.FLOAT;
        }

        @Override       // override all the functions of the real DcMotor class that touch hardware
        public void setPower(double power) {
            mPower = power;
            mOpMode.telemetry.addData(mName, " power: " + String.valueOf(mPower));
        }

        public double getPower() {
            return mPower;
        }

        public void close() {
            mOpMode.telemetry.addData(mName, " close();");
        }

        public boolean isBusy() {
            return false;
        }

        public void setPowerFloat() {
            mPowerFloat = true;
            mOpMode.telemetry.addData(mName, " setPowerFloat();");
        }

        public boolean getPowerFloat() {
            return mPowerFloat;
        }

        public void setMaxSpeed(int encoderTicksPerSecond)
        {
            mMaxSpeed = encoderTicksPerSecond;
            mOpMode.telemetry.addData(mName, "maxSpeed: " + String.valueOf(encoderTicksPerSecond));
        }

        public int getMaxSpeed() { return mMaxSpeed; }

        public void setTargetPosition(int position) {
            mTargetPosition = position;
            mOpMode.telemetry.addData(mName, "target: " + String.valueOf(position));
        }

        public int getTargetPosition() {
            return mTargetPosition;
        }

        public int getCurrentPosition() {
            return mTargetPosition;
        }

        public void setMode(DcMotor.RunMode mode) {
            this.mMode = mode;
            mOpMode.telemetry.addData(mName, "run mode: " + String.valueOf(mode));
        }

        public DcMotor.RunMode getMode() {
            return this.mMode;
        }

        public void setDirection(Direction direction)
        {
            mDirection = direction;
            mOpMode.telemetry.addData(mName, "direction: " + String.valueOf(direction));
        }

        public Direction getDirection() { return mDirection; }

        public String getConnectionInfo() {
            return mName + " port: unknown";
        }

        public DcMotorController getController()
        {
            return null;
        }

        public void resetDeviceConfigurationForOpMode() {
            mOpMode.telemetry.addData(mName, "resetDeviceConfigurationForOpMode: ");
        }

        public int getPortNumber()
        {
            return 0;
        }

        public ZeroPowerBehavior getZeroPowerBehavior()
        {
            return mZeroPowerBehavior;
        }

        public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior)
        {
            mZeroPowerBehavior = zeroPowerBehavior;
            mOpMode.telemetry.addData(mName, "zeroPowerBehavior: " + String.valueOf(zeroPowerBehavior));
        }

        public int getVersion() { return 0; }

        public HardwareDevice.Manufacturer getManufacturer() { return Manufacturer.Unknown; }

        public String getDeviceName() { return "AutoLib_TestMotor: " + mName; }


    }*/

    // a dummy Servo that just logs commands we send to it --
    // useful for testing Servo code when you don't have real hardware handy
    static public class TestServo implements Servo {
        OpMode mOpMode;     // needed for logging data
        String mName;       // string id of this servo
        double mPosition;   // current target position
        Direction mDirection;
        double mScaleMin;
        double mScaleMax;

        public TestServo(String name, OpMode opMode) {
            super();     // init base class (real DcMotor) with dummy data
            mOpMode = opMode;
            mName = name;
            mPosition = 0.0;
        }

        @Override       // this function overrides the setPower() function of the real DcMotor class
        public void setPosition(double position) {
            mPosition = position;
            mOpMode.telemetry.addData(mName, " position: " + String.valueOf(mPosition));
            mDirection = Direction.FORWARD;
        }

        @Override       // this function overrides the getPower() function of the real DcMotor class
        public double getPosition() {
            return mPosition;
        }

        @Override       // override all other functions of Servo that touch the hardware
        public String getConnectionInfo() {
            return mName + " port: unknown";
        }

        public HardwareDevice.Manufacturer getManufacturer() { return Manufacturer.Unknown; }

        public void resetDeviceConfigurationForOpMode() {
            mOpMode.telemetry.addData(mName, "resetDeviceConfigurationForOpMode: ");
        }

        public Servo.Direction getDirection() { return mDirection; }
        public void setDirection(Servo.Direction direction) {
            mDirection = direction;
            mOpMode.telemetry.addData(mName, "direction: " + String.valueOf(mDirection));
        }

        public void scaleRange(double min, double max)
        {
            mScaleMin = min;
            mScaleMax = max;
        }

        public ServoController getController() { return null; }
        public String getDeviceName() { return "AutoLib_TestServo: " + mName; }
        public int getPortNumber()
        {
            return 0;
        }
        public int getVersion() { return 0; }

        public void close() {}

    }

    // define interface to Factory that creates various kinds of hardware objects
    static public interface HardwareFactory {
        public DcMotor getDcMotor(String name);
        public Servo getServo(String name);
    }

    // this implementation generates test-hardware objects that just log data
  /*  static public class TestHardwareFactory implements HardwareFactory {
        OpMode mOpMode;     // needed for logging data

        public TestHardwareFactory(OpMode opMode) {
            mOpMode = opMode;
        }

        //public DcMotor getDcMotor(String name){
          //  return new TestMotor(name, mOpMode);
        //}

        public Servo getServo(String name){
            return new TestServo(name, mOpMode);
        }
    }*/

    // this implementation gets real hardware objects from the hardwareMap of the given OpMode
    static public class RealHardwareFactory implements HardwareFactory {
        OpMode mOpMode;     // needed for access to hardwareMap

        public RealHardwareFactory(OpMode opMode) {
            mOpMode = opMode;
        }

        public DcMotor getDcMotor(String name){
            DcMotor motor = null;
            try {
                motor = mOpMode.hardwareMap.dcMotor.get(name);
            }
            catch (Exception e) {
                // okay -- just return null (absent) for this motor
            }

            // just to make sure - a previous OpMode may have set it differently ...
            if (motor != null)
                motor.setDirection(DcMotor.Direction.FORWARD);

            return motor;
        }

        public Servo getServo(String name){
            Servo servo = null;
            try {
                servo = mOpMode.hardwareMap.servo.get(name);
            }
            catch (Exception e) {
                // okay - just return null (absent) for this servo
            }

            // just to make sure - a previous OpMode may have set it differently ...
            if (servo != null)
                servo.setDirection(Servo.Direction.FORWARD);
            
            return servo;
        }
    }


}



