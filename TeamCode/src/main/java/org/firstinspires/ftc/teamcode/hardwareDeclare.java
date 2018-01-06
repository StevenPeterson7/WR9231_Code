package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

public class hardwareDeclare{
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    // Declare the motor matrix
    public DcMotor[] motors = new DcMotor[4];
    //public DcMotor[] testMotor = new DcMotor [1];

    // Declare the two lifter motors
    public DcMotor[] glyphLift = new DcMotor [2];
    public Servo[] glyphLiftArms = new Servo[2];
    public Servo whacker;
    // Declare Servo Arm
  //  public CRServo[] servos = new CRServo[2];
    // Declare the Color Sensor
    public ColorSensor ColorSensor;
    public Servo placer;
  //  public ModernRoboticsI2cGyro gyroH;
    //public SensorLib.CorrectedMRGyro mGyro;


    public hardwareDeclare(OpMode opmode) {

        //Drive Motors
        //testMotor[0] = opmode.hardwareMap.dcMotor.get("test");
        motors[2] = opmode.hardwareMap.dcMotor.get("frontLeft");
        motors[0] = opmode.hardwareMap.dcMotor.get("frontRight");
        motors[3] = opmode.hardwareMap.dcMotor.get("backLeft");
        motors[1] = opmode.hardwareMap.dcMotor.get("backRight");
        motors[2].setDirection(DcMotor.Direction.REVERSE);
        motors[3].setDirection(DcMotor.Direction.REVERSE);
        motors[0].setDirection(DcMotor.Direction.FORWARD);
        motors[1].setDirection(DcMotor.Direction.FORWARD);

        glyphLift[0]=opmode.hardwareMap.dcMotor.get("glyphSpin");
        glyphLift[1]=opmode.hardwareMap.dcMotor.get("glyphLift");



        imu = opmode.hardwareMap.get(BNO055IMU.class, "imu");





        glyphLiftArms[0] = opmode.hardwareMap.servo.get("leftArm");
        glyphLiftArms[1] = opmode.hardwareMap.servo.get("rightArm");
        glyphLiftArms[1].setDirection(REVERSE);
        glyphLiftArms[0].scaleRange(0,0.844);
        glyphLiftArms[1].scaleRange(0,0.8444);

        placer=opmode.hardwareMap.servo.get("placer");


        // Ball Lifter and Launcher Motors
        /*liftMotors[0] = opmode.hardwareMap.dcMotor.get("motorSweeper");
        liftMotors[1] = opmode.hardwareMap.dcMotor.get("motorBelt");
        liftMotors[2] = opmode.hardwareMap.dcMotor.get("motorLaunch");
        liftMotors[0].setDirection(DcMotor.Direction.REVERSE);
        liftMotors[1].setDirection(DcMotor.Direction.REVERSE);
        liftMotors[2].setDirection(DcMotor.Direction.REVERSE);

        servos[0] = opmode.hardwareMap.crservo.get("servoLeft");
        servos[1] = opmode.hardwareMap.crservo.get("servoRight");*/

          ColorSensor = (ColorSensor) opmode.hardwareMap.colorSensor.get("cs");
          whacker = opmode.hardwareMap.servo.get("whacker");

        /*gyroH = (ModernRoboticsI2cGyro) opmode.hardwareMap.gyroSensor.get("gyro");

        mGyro = new SensorLib.CorrectedMRGyro(gyroH);
        mGyro.calibrate();*/
    }
}
