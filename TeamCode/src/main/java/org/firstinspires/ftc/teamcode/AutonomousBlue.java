package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousBlue",group="Autonomous")
public class AutonomousBlue extends OpMode{
    // Declare the motor matrix
    private DcMotor[][] motors = new DcMotor[2][2];
    // Set default power to 100%
    private float motorPower = 1.f;
    @Override
    public void init() {
        int F = 0;
        int B = 1;
        int L = 0;
        int R = 1;
        motors[F][L] = hardwareMap.dcMotor.get("motorFrontLeft");
        motors[F][R] = hardwareMap.dcMotor.get("motorFrontRight");
        motors[B][L] = hardwareMap.dcMotor.get("motorBackLeft");
        motors[B][R] = hardwareMap.dcMotor.get("motorBackRight");

        motors[F][L].setDirection(DcMotor.Direction.FORWARD);
        motors[F][R].setDirection(DcMotor.Direction.REVERSE);
        motors[B][L].setDirection(DcMotor.Direction.REVERSE);
        motors[B][R].setDirection(DcMotor.Direction.FORWARD);

        for(DcMotor[] motor : motors){
            // Set left motor power
            motor[0].setPower(.25);
            // Set right motor power
            motor[1].setPower(.25);
        }
        //for 2 seconds
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        for(DcMotor[] motor : motors){
            // Set left motor power
            motor[0].setPower(1);
            // Set right motor power
            motor[1].setPower(0);
        //270 deg turn
        }
        //for 2 seconds
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        for(DcMotor[] motor : motors){
            // Set left motor power
            motor[0].setPower(.25);
            // Set right motor power
            motor[1].setPower(.25);
        }
        //for 2 seconds
        try {
            Thread.sleep(2250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        for(DcMotor[] motor : motors){
            // Set left motor power
            motor[0].setPower(0);
            // Set right motor power
            motor[1].setPower(0);
        }


    }

    @Override
    public void loop() {

    }
}
