package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.annotation.Annotation;

/**
 * Created by Robotics on 11/9/2016.
 */
@TeleOp (name = "Pre Match Test", group = "TeleOp")
public class PreMatchTest extends OpMode {
    // Declare the motor matrix
    private DcMotor[][] motors = new DcMotor[2][2];
    // Set default power to 100%
    private float motorPower = 1.f;
    int F = 0;
    int B = 1;
    int L = 0;
    int R = 1;

    @Override
    public void init() {
        //initialize our motors the same as our other code
        motors[F][L] = hardwareMap.dcMotor.get("motorFrontLeft");
        motors[F][R] = hardwareMap.dcMotor.get("motorFrontRight");
        motors[B][L] = hardwareMap.dcMotor.get("motorBackLeft");
        motors[B][R] = hardwareMap.dcMotor.get("motorBackRight");
        motors[F][L].setDirection(DcMotor.Direction.FORWARD);
        motors[F][R].setDirection(DcMotor.Direction.REVERSE);
        motors[B][L].setDirection(DcMotor.Direction.REVERSE);
        motors[B][R].setDirection(DcMotor.Direction.FORWARD);

    }
    boolean startPush = false;
    @Override
    public void loop() {
        //gives telemetry feedback based on buttons pressed
        if(gamepad1.a)telemetry.addData("Buttons Pressed", "A");
        if(gamepad1.b)telemetry.addData("Buttons Pressed", "B");
        if(gamepad1.x)telemetry.addData("Buttons Pressed", "X");
        if(gamepad1.y)telemetry.addData("Buttons Pressed", "Y");
        if(gamepad1.left_bumper)telemetry.addData("Buttons Pressed", "LB");
        if(gamepad1.right_bumper)telemetry.addData("Buttons Pressed", "RB");
        if(gamepad1.left_trigger>0)telemetry.addData("Buttons Pressed", "LT");
        if(gamepad1.right_trigger>0)telemetry.addData("Buttons Pressed", "RT");
        if(gamepad1.left_stick_button)telemetry.addData("Buttons Pressed", "LSB");
        if(gamepad1.right_stick_button)telemetry.addData("Buttons Pressed", "RSB");
        //wait for user input to start the check through
        if(gamepad1.right_stick_button && gamepad1.left_stick_button) startPush=true;
        if(startPush){
            //Begin Check
            telemetry.addData("Testing Active",1);
            //test front left motor
            telemetry.addData("Testing Motor", "Front Left");
            motors[F][L].setPower(.25);
            try {
                wait(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            motors[F][L].setPower(0);
            telemetry.addData("Testing Active",1);
            //test front right motor
            telemetry.addData("Testing Motor", "Front Right");
            motors[F][R].setPower(.25);
            try {
                wait(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            motors[F][R].setPower(0);
            telemetry.addData("Testing Active",1);
            //test back left motor
            telemetry.addData("Testing Motor", "Back Left");
            motors[B][L].setPower(.25);
            try {
                wait(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            motors[B][L].setPower(0);
            //test back right motor
            telemetry.addData("Testing Motor", "Back Right");
            motors[B][R].setPower(.25);
            try {
                wait(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            motors[B][R].setPower(0);
            startPush=false;
            telemetry.clear();

        }

    }
}
