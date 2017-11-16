package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RatBotTeleOp:2P", group="TeleOp")
public class RatBotTeleOp_2p extends OpMode{

    hardwareDeclare2 hw;
    private float motorPower = 1.f;

    @Override
    public void init(){

        // Get our hardware
        hw = new hardwareDeclare2(this);


    }

    @Override
    public void loop(){
        telemetry.addData("motor stuff", hw.motors[0].getPower());
        double powerToLeftMotor = -gamepad1.left_stick_y * motorPower;
        telemetry.addData("power to  left motor:", powerToLeftMotor);
        double powerToRightMotor = -gamepad1.right_stick_y * motorPower;
        telemetry.addData("power to  right motor:", powerToRightMotor);
        hw.motors[2].setPower(powerToLeftMotor);
        hw.motors[3].setPower(powerToLeftMotor);

        hw.motors[0].setPower(powerToRightMotor);
        // Set right motor power
        hw.motors[1].setPower(powerToRightMotor);

        hw.whacker.setPosition(gamepad2.left_stick_y);

        telemetry.addData("position ",hw.whacker.getPosition());



    }
    @Override
    public void stop(){

    }
}

