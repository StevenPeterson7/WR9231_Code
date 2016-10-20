package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="SimpleTeleOp", group="TeleOp")
public class SimpleTeleOp extends OpMode{
    // Declare the motor matrix
    private DcMotor[][] motors = new DcMotor[2][2];

    @Override
    public void init(){
        // Set up the motor matrix (that sounds cool)
        motors[0][0] = hardwareMap.dcMotor.get("motorFrontLeft");
        motors[0][1] = hardwareMap.dcMotor.get("motorFrontRight");
        motors[1][0] = hardwareMap.dcMotor.get("motorBackLeft");
        motors[1][1] = hardwareMap.dcMotor.get("motorBackRight");
        // The motors on the left side of the robot need to be in reverse mode
        for(DcMotor[] motor : motors){
            motor[0].setDirection(DcMotor.Direction.REVERSE);
        }
        // Being explicit never hurt anyone, right?
        for(DcMotor[] motor : motors){
            motor[1].setDirection(DcMotor.Direction.FORWARD);
        }
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }
    @Override
    public void loop(){
        // Loop through front and back motors
        for(DcMotor[] motor : motors){
            // Set left motor power
            motor[0].setPower(-gamepad1.left_stick_y);
            // Set right motor power
            motor[1].setPower(-gamepad1.right_stick_y);
        }
    }
    @Override
    public void stop(){

    }
}
