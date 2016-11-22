package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousRed",group="Autonomous")
public class AutonomousRed extends OpMode{
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

        motors[F][L].setDirection(DcMotor.Direction.REVERSE);
        motors[F][R].setDirection(DcMotor.Direction.FORWARD);
        motors[B][L].setDirection(DcMotor.Direction.FORWARD);
        motors[B][R].setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start(){

        //Wait 10 Seconds
        for(DcMotor[] motor : motors){
            // Set left motor power
            motor[0].setPower(0);
            // Set right motor power
            motor[1].setPower(0);
        }
        try {
            Thread.sleep(10000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //Drive forward for 1.5 seconds
        for(DcMotor[] motor : motors) {
            // Set left motor power
            motor[0].setPower(.5);
            // Set right motor power
            motor[1].setPower(.5);
        }
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

/*        //270 Degree Turn - 1.5 seconds
        for(DcMotor[] motor : motors){
            // Set left motor power
            motor[0].setPower(0);
            // Set right motor power
            motor[1].setPower(1);
        }
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/

/*        //drive forward for 2.25 seconds
        for(DcMotor[] motor : motors) {
            // Set left motor power
            motor[0].setPower(.25);
            // Set right motor power
            motor[1].setPower(.25);
        }
        try {
            Thread.sleep(2250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/

        //Stop
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
