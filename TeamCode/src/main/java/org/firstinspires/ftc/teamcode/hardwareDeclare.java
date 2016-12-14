package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class hardwareDeclare{
    // Declare the motor matrix
    public DcMotor[] motors = new DcMotor[4];
    // Declare the two lifter motors
    public DcMotor[] liftMotors = new DcMotor [3];

    public hardwareDeclare(OpMode opmode) {

        motors[2] = opmode.hardwareMap.dcMotor.get("motorFrontLeft");
        motors[0] = opmode.hardwareMap.dcMotor.get("motorFrontRight");
        motors[3] = opmode.hardwareMap.dcMotor.get("motorBackLeft");
        motors[1] = opmode.hardwareMap.dcMotor.get("motorBackRight");

        motors[2].setDirection(DcMotor.Direction.REVERSE);
        motors[0].setDirection(DcMotor.Direction.FORWARD);
        motors[3].setDirection(DcMotor.Direction.FORWARD);
        motors[1].setDirection(DcMotor.Direction.REVERSE);

        liftMotors[0] = opmode.hardwareMap.dcMotor.get("motorSweeper");
        liftMotors[1] = opmode.hardwareMap.dcMotor.get("motorBelt");
        liftMotors[2] = opmode.hardwareMap.dcMotor.get("motorLaunch");

        liftMotors[0].setDirection(DcMotor.Direction.REVERSE);
        liftMotors[1].setDirection(DcMotor.Direction.REVERSE);
        liftMotors[2].setDirection(DcMotor.Direction.REVERSE);

    }
}
