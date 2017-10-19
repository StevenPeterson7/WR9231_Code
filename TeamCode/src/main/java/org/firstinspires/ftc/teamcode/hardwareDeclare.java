package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class hardwareDeclare{
    // Declare the motor matrix
    public DcMotor[] motors = new DcMotor[4];
    //public DcMotor[] testMotor = new DcMotor [1];

    // Declare the two lifter motors
    public DcMotor[] glyphLift = new DcMotor [2];
    public Servo[] glyphLiftArms = new Servo[2];
    // Declare Servo Arm
  //  public CRServo[] servos = new CRServo[2];
    // Declare the Color Sensor
    //public ModernRoboticsI2cColorSensor mColorSensor;
  //  public ModernRoboticsI2cGyro gyroH;
    //public SensorLib.CorrectedMRGyro mGyro;


    public hardwareDeclare(OpMode opmode) {

        //Drive Motors
        //testMotor[0] = opmode.hardwareMap.dcMotor.get("test");
        motors[2] = opmode.hardwareMap.dcMotor.get("motorFrontLeft");
        motors[0] = opmode.hardwareMap.dcMotor.get("motorFrontRight");
        motors[3] = opmode.hardwareMap.dcMotor.get("motorBackLeft");
        motors[1] = opmode.hardwareMap.dcMotor.get("motorBackRight");
        motors[2].setDirection(DcMotor.Direction.REVERSE);
        motors[3].setDirection(DcMotor.Direction.FORWARD);
        motors[0].setDirection(DcMotor.Direction.FORWARD);
        motors[1].setDirection(DcMotor.Direction.FORWARD);

        glyphLift[0]=opmode.hardwareMap.dcMotor.get("glyphSpin");
        glyphLift[1]=opmode.hardwareMap.dcMotor.get("glyphLift");

        glyphLiftArms[0] = opmode.hardwareMap.servo.get("leftArm");
        glyphLiftArms[1] = opmode.hardwareMap.servo.get("rightArm");
        // Ball Lifter and Launcher Motors
        /*liftMotors[0] = opmode.hardwareMap.dcMotor.get("motorSweeper");
        liftMotors[1] = opmode.hardwareMap.dcMotor.get("motorBelt");
        liftMotors[2] = opmode.hardwareMap.dcMotor.get("motorLaunch");
        liftMotors[0].setDirection(DcMotor.Direction.REVERSE);
        liftMotors[1].setDirection(DcMotor.Direction.REVERSE);
        liftMotors[2].setDirection(DcMotor.Direction.REVERSE);

        servos[0] = opmode.hardwareMap.crservo.get("servoLeft");
        servos[1] = opmode.hardwareMap.crservo.get("servoRight");

        mColorSensor = (ModernRoboticsI2cColorSensor) opmode.hardwareMap.colorSensor.get("cs");

        gyroH = (ModernRoboticsI2cGyro) opmode.hardwareMap.gyroSensor.get("gyro");

        mGyro = new SensorLib.CorrectedMRGyro(gyroH);
        mGyro.calibrate();*/
    }
}
