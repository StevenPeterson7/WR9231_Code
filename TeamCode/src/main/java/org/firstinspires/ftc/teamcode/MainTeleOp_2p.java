package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

@TeleOp(name="MainTeleOp:2P", group="TeleOp")
public class MainTeleOp_2p extends OpMode{

    hardwareDeclare hw;
    private float motorPower = 1.f;
    double lPos=1;
    double rPos=1;

    @Override
    public void init(){

        // Get our hardware
        hw = new hardwareDeclare(this);
        hw.glyphLiftArms[0].setPosition(lPos);
        hw.glyphLiftArms[1].setPosition(rPos);
        hw.ColorSensor.enableLed(false);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        hw.imu.initialize(parameters);


    }

    @Override
    public void loop(){

        hw.whacker.setPosition(1);
        // Check for and apply changes to motorPower
        if(gamepad1.right_bumper) motorPower = 0.25f;
        if(gamepad1.left_bumper) motorPower = 1.f;

        // Make sure the motorPower stays between 0-100% inclusive
        if(motorPower<0.f) motorPower = 0.0f;
        if(motorPower>1.f) motorPower = 1.0f;

        // Tell what the power is
        telemetry.addData("Motor power (%)", motorPower);


        // Set left motor power
        double powerToLeftMotor = -gamepad1.left_stick_y * motorPower;
        //apply left motor power
        hw.motors[2].setPower(powerToLeftMotor);
        hw.motors[3].setPower(powerToLeftMotor);
        telemetry.addData("position motor 3", hw.motors[3].getCurrentPosition());

        // Set right motor power
        double powerToRightMotor = -gamepad1.right_stick_y* motorPower;
        //apply right motor power
        hw.motors[0].setPower(powerToRightMotor);
        hw.motors[1].setPower(powerToRightMotor);

        telemetry.addData("power to  left motor:", powerToLeftMotor);
        telemetry.addData("power to  right motor:", powerToRightMotor);

        //}
        telemetry.addData("glyph lift: ", gamepad2.left_stick_y );
        telemetry.addData("glyph lift pos: ", hw.glyphLift[1].getCurrentPosition());

        if(gamepad2.left_stick_y!=0){
            /*the power to the glyph lift is cubed so that it will be easy to make fine adjustments
            when the glyph lift is close to the desired destination but it can also be speed up to
            full speed for bigger adjustments
             */
            hw.glyphLift[1].setPower(pow(gamepad2.left_stick_y,3));
        }else{
            hw.glyphLift[1].setPower(0);
        }

        telemetry.addData("glyph spin: ", gamepad2.right_stick_x );
        telemetry.addData("glyph spin pos: ", hw.glyphLift[0].getCurrentPosition());
        if(hw.glyphLift[0].getCurrentPosition() <= 475 && gamepad2.right_stick_x > 0){
            hw.glyphLift[0].setPower(gamepad2.right_stick_x * 0.2);
        }else if(hw.glyphLift[0].getCurrentPosition()>=-475 && gamepad2.right_stick_x < 0) {
            hw.glyphLift[0].setPower(gamepad2.right_stick_x * 0.2);
        }else {
            hw.glyphLift[0].setPower(0);
        }


        telemetry.addData("servo1 position:", hw.glyphLiftArms[0].getPosition());
        telemetry.addData("servo2 position:", hw.glyphLiftArms[1].getPosition());

        //
         telemetry.addData("gyro: ", hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);


        //if the arm positions are not within set bounds, reset them
        if(lPos<0){
            lPos=0;
        }
        if(rPos<0){
            rPos=0;
        }
        if(lPos>1){
                lPos=1;
        }
        if(rPos>1){
                rPos=1;
        }
        //slowly close the arm when the bumper is pushed
        if(gamepad2.right_bumper){
            rPos-=0.07;
        }
        if(gamepad2.left_bumper){
            lPos-=0.07;
        }

        /*
        when the left button is pressed, move the block to the left but
        make sure that there is a constant pressure on the block
         */
        if(gamepad2.dpad_left){
            if(rPos<1) {
                rPos += 0.11;
            }
            if(lPos>0) {
                lPos -= 0.07;
            }
        }
        /*
        do the opposite for the right side
         */
        if(gamepad2.dpad_right){
            if(lPos<1) {
                lPos += 0.11;
            }
            if(rPos>0) {
                rPos -= 0.07;
            }
        }

        //slowly open the arms when the trigger is pulled with the rate depending on how much the triggers are pulled
        lPos+=gamepad2.left_trigger/15;
        rPos+=gamepad2.right_trigger/15;

        //apply the new positions
        hw.glyphLiftArms[0].setPosition(lPos);
        hw.glyphLiftArms[1].setPosition(rPos);

        telemetry.addData("right trigger", gamepad2.right_trigger);


      /*  if(gamepad1.a || gamepad2.a){
           // hw.testMotor[0].setPower(1.0);
            hw.testMotor[0].setPower(1.0);
        }
        else{
           hw.testMotor[0].setPower(0.0);
        }*/
        /*

        if(gamepad1.b || gamepad2.b){
            hw.liftMotors[0].setPower(1.0);
            hw.liftMotors[1].setPower(1.0);
        }
        else{
            hw.liftMotors[0].setPower(0.0);
            hw.liftMotors[1].setPower(0.0);
        }

        if(gamepad1.x){
            hw.servos[0].setPower(-1.0);
            hw.servos[1].setPower(-1.0);
        }
        else if(gamepad1.y){
            hw.servos[0].setPower(1.0);
            hw.servos[1].setPower(1.0);
        }
        else{
            hw.servos[0].setPower(0.0);
            hw.servos[1].setPower(0.0);
        }*/



    }
    @Override
    public void stop(){
    }
}

