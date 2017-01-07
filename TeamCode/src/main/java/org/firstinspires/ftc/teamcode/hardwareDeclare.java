package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;

public class hardwareDeclare{
    // Declare the motor matrix
    public DcMotor[] motors = new DcMotor[4];
    // Declare the two lifter motors
    public DcMotor[] liftMotors = new DcMotor [3];
    // Declare Servo Arm
    public Servo[] servos = new Servo [2];
    // Declare the Color Sensor
    public ModernRoboticsI2cColorSensor mColorSensor;


    public hardwareDeclare(OpMode opmode) {

        //Drive Motors
        motors[2] = opmode.hardwareMap.dcMotor.get("motorFrontLeft");
        motors[0] = opmode.hardwareMap.dcMotor.get("motorFrontRight");
        motors[3] = opmode.hardwareMap.dcMotor.get("motorBackLeft");
        motors[1] = opmode.hardwareMap.dcMotor.get("motorBackRight");
        motors[2].setDirection(DcMotor.Direction.REVERSE);
        motors[0].setDirection(DcMotor.Direction.FORWARD);
        motors[3].setDirection(DcMotor.Direction.FORWARD);
        motors[1].setDirection(DcMotor.Direction.REVERSE);

        // Ball Lifter and Launcher Motors
        liftMotors[0] = opmode.hardwareMap.dcMotor.get("motorSweeper");
        liftMotors[1] = opmode.hardwareMap.dcMotor.get("motorBelt");
        liftMotors[2] = opmode.hardwareMap.dcMotor.get("motorLaunch");
        liftMotors[0].setDirection(DcMotor.Direction.REVERSE);
        liftMotors[1].setDirection(DcMotor.Direction.REVERSE);
        liftMotors[2].setDirection(DcMotor.Direction.REVERSE);

        servos[0] = opmode.hardwareMap.servo.get("servoLeft");
        servos[1] = opmode.hardwareMap.servo.get("servoRight");

//        servos[0] = opmode.hardwareMap.crservo.get("servoLeft");
//        servos[1] = opmode.hardwareMap.crservo.get("servoRight");

        mColorSensor = (ModernRoboticsI2cColorSensor) opmode.hardwareMap.colorSensor.get("cs");

    }
}
