package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousBlue",group="Autonomous")
public class AutonomousBlue extends OpMode{
    // Declare the motor matrix
    private DcMotor[][] motors = new DcMotor[2][2];
    // Declare the two lifter motors
    private DcMotor[] liftMotors = new DcMotor [3];
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
    public void start() {

    }

    @Override
    public void loop() {
        //Wait 10 Seconds
        for(DcMotor[] motor : motors){
            // Set left motor power
            motor[0].setPower(0);
            // Set right motor power
            motor[1].setPower(0);
        }
//        try {
//            Thread.sleep(10000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }

        for(DcMotor[] motor : motors){
            // Set left motor power
            motor[0].setPower(1.0);
            // Set right motor power
            motor[1].setPower(1.0);
        }

        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        for(DcMotor[] motor : motors){
            // Set left motor power
            motor[0].setPower(0.0);
            // Set right motor power
            motor[1].setPower(1.0);
        }

        try {
            Thread.sleep(1200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        liftMotors[1].setPower(1.0);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        liftMotors[1].setPower(0.0);

        liftMotors[2].setPower(1.0);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        liftMotors[2].setPower(0.0);

        liftMotors[1].setPower(1.0);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        liftMotors[1].setPower(0.0);

        //Stop
        for(DcMotor[] motor : motors) {
            // Set left motor power
            motor[0].setPower(0);
            // Set right motor power
            motor[1].setPower(0);
        }
    }
}
