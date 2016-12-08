package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="SimpleTeleOp", group="TeleOp")
public class SimpleTeleOp extends OpMode{
    // Declare the motor matrix
    private DcMotor[][] motors = new DcMotor[2][2];
    // Declare the two lifter motors
    private DcMotor[] liftMotors = new DcMotor [3];
    // Set default power to 100%
    private float motorPower = 1.f;

    //private boolean liftToggle[] = new boolean[2];

    @Override
    public void init(){

        int F = 0;
        int B = 1;
        int L = 0;
        int R = 1;

        // Set up the motor matrix (that sounds cool)
        motors[F][L] = hardwareMap.dcMotor.get("motorFrontLeft");
        motors[F][R] = hardwareMap.dcMotor.get("motorFrontRight");
        motors[B][L] = hardwareMap.dcMotor.get("motorBackLeft");
        motors[B][R] = hardwareMap.dcMotor.get("motorBackRight");

        motors[F][L].setDirection(DcMotor.Direction.REVERSE);
        motors[F][R].setDirection(DcMotor.Direction.FORWARD);
        motors[B][L].setDirection(DcMotor.Direction.FORWARD);
        motors[B][R].setDirection(DcMotor.Direction.REVERSE);

        liftMotors[0] = hardwareMap.dcMotor.get("motorSweeper");
        liftMotors[1] = hardwareMap.dcMotor.get("motorBelt");
        liftMotors[2] = hardwareMap.dcMotor.get("motorLaunch");

        liftMotors[0].setDirection(DcMotor.Direction.REVERSE);
        liftMotors[1].setDirection(DcMotor.Direction.REVERSE);
        liftMotors[2].setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

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
        for(DcMotor[] motor : motors){
            // Set left motor power
            motor[0].setPower(-gamepad1.left_stick_y*motorPower);
            // Set right motor power
            motor[1].setPower(-gamepad1.right_stick_y*motorPower);
        }

        if(gamepad1.a){
            liftMotors[1].setPower(1.0);
        }
        else{
            liftMotors[1].setPower(0.0);
        }

        if(gamepad1.b){
            liftMotors[0].setPower(1.0);
            liftMotors[2].setPower(1.0);
        }
        else{
            liftMotors[0].setPower(0.0);
            liftMotors[2].setPower(0.0);
        }

    }
    @Override
    public void stop(){

    }
}

