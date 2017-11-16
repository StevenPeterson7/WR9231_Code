package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class hardwareDeclare2 {
    // Declare the motor matrix
    public DcMotor[] motors = new DcMotor[4];
    //public DcMotor[] testMotor = new DcMotor [1];
    public Servo whacker;
    public ModernRoboticsI2cColorSensor mColorSensor;
    // Declare the two lifter motors
   /* public DcMotor[] glyphLift = new DcMotor [2];
    public DcMotor [] relicArm = new DcMotor[2];
    public Servo[] glyphLiftArms = new Servo[2];*/
    // Declare Servo Arm
  //  public CRServo[] servos = new CRServo[2];
    // Declare the Color Sensor
    //public ModernRoboticsI2cColorSensor mColorSensor;
  //  public ModernRoboticsI2cGyro gyroH;
    //public SensorLib.CorrectedMRGyro mGyro;


    public hardwareDeclare2(OpMode opmode) {

        //Drive Motors
        //testMotor[0] = opmode.hardwareMap.dcMotor.get("test");
        motors[2] = opmode.hardwareMap.dcMotor.get("motorFrontLeft");
        motors[0] = opmode.hardwareMap.dcMotor.get("motorFrontRight");
        motors[3] = opmode.hardwareMap.dcMotor.get("motorBackLeft");
        motors[1] = opmode.hardwareMap.dcMotor.get("motorBackRight");
        motors[2].setDirection(DcMotor.Direction.FORWARD);
        motors[3].setDirection(DcMotor.Direction.FORWARD);
        motors[0].setDirection(DcMotor.Direction.FORWARD);
        motors[1].setDirection(DcMotor.Direction.FORWARD);
        mColorSensor = (ModernRoboticsI2cColorSensor) opmode.hardwareMap.colorSensor.get("cs");
        whacker = opmode.hardwareMap.servo.get("whacker");


    }
}
