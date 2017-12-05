package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.abs;

@TeleOp(name="MainTeleOp:2P", group="TeleOp")
public class MainTeleOp_2p extends OpMode{

    double lPos=0;
    double rPos=0;

    hardwareDeclare hw;
    private float motorPower = 1.f;

    @Override
    public void init(){

        // Get our hardware
        hw = new hardwareDeclare(this);
        hw.glyphLiftArms[0].setPosition(lPos);
        hw.glyphLiftArms[1].setPosition(rPos);
        hw.ColorSensor.enableLed(false);


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



        // Loop through front and back motors96
       // for(DcMotor motor : hw.motors) {
            // Set left motor power
            double powerToLeftMotor = -gamepad1.left_stick_y * motorPower;
            telemetry.addData("power to  left motor:", powerToLeftMotor);
            double powerToRightMotor = -gamepad1.right_stick_y * motorPower;
            telemetry.addData("power to  right motor:", powerToRightMotor);
            hw.motors[2].setPower(powerToLeftMotor);
            hw.motors[3].setPower(powerToLeftMotor);

            hw.motors[0].setPower(powerToRightMotor);
            // Set right motor power
            hw.motors[1].setPower(powerToRightMotor);
        //}
        telemetry.addData("glyph lift: ", gamepad2.left_stick_y );
        telemetry.addData("glyph lift pos: ", hw.glyphLift[1].getCurrentPosition());

        if(gamepad2.left_stick_y!=0){
            hw.glyphLift[1].setPower(gamepad2.left_stick_y);
        }else{
            hw.glyphLift[1].setPower(0);
        }

        telemetry.addData("glyph spin: ", gamepad2.right_stick_x );
        telemetry.addData("glyph spin pos: ", hw.glyphLift[0].getCurrentPosition());
        if(hw.glyphLift[0].getCurrentPosition()<=450&&gamepad2.right_stick_x>0){
            hw.glyphLift[0].setPower(gamepad2.right_stick_x * 0.2);
        }else if(hw.glyphLift[0].getCurrentPosition()>=-450 && gamepad2.right_stick_x < 0) {
            hw.glyphLift[0].setPower(gamepad2.right_stick_x * 0.2);
        }else {
            hw.glyphLift[0].setPower(0);
        }


        telemetry.addData("servo1 position:", hw.glyphLiftArms[0].getPosition());
        telemetry.addData("servo2 position:", hw.glyphLiftArms[1].getPosition());

       /* if(gamepad2.a){

            lPos=1;
            rPos=1;
        }else{

            lPos=0;
            rPos=0;

        }*/
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
            lPos-=gamepad2.right_trigger/20;
            rPos-=gamepad2.right_trigger/20;

            lPos+=gamepad2.left_trigger/20;
            rPos+=gamepad2.left_trigger/20;
        //we can change it to use the stick positions to either set position or set change in position
        //left stick could control the left arm

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

