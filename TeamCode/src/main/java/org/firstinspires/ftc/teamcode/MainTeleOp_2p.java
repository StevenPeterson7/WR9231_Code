package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MainTeleOp:2P", group="TeleOp")
public class MainTeleOp_2p extends OpMode{

    hardwareDeclare hw;
    private float motorPower = 1.f;

    @Override
    public void init(){

        // Get our hardware
        hw = new hardwareDeclare(this);

    }

    @Override
    public void loop(){
        // Check for and apply changes to motorPower
        if(gamepad1.right_bumper) motorPower = 0.25f;
        if(gamepad1.left_bumper) motorPower = 1.f;

        // Make sure the motorPower stays between 0-100% inclusive
        if(motorPower<0.f) motorPower = 0.0f;
        if(motorPower>1.f) motorPower = 1.0f;

        // Tell what the power is
        telemetry.addData("Motor power (%)", motorPower);

        // Loop through front and back motors
        for(DcMotor motor : hw.motors){
            // Set left motor power
            hw.motors[2].setPower(-gamepad1.left_stick_y*motorPower);
            hw.motors[3].setPower(-gamepad1.left_stick_y*motorPower);
            // Set right motor power
            hw.motors[0].setPower(-gamepad1.right_stick_y*motorPower);
            hw.motors[1].setPower(-gamepad1.right_stick_y*motorPower);
        }

        if(gamepad1.a || gamepad2.a){
            hw.liftMotors[2].setPower(1.0);
        }
        else{
            hw.liftMotors[2].setPower(0.0);
        }

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
        }
        
    }
    @Override
    public void stop(){

    }
}

