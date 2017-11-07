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
        if(gamepad1.left_stick_y >0){
            hw.motors[2].setPower(1);
            hw.motors[3].setPower(1);
        }
        if(gamepad1.right_stick_y >0){
            hw.motors[0].setPower(1);
            hw.motors[1].setPower(1);
        }
        if(gamepad1.left_stick_y <0){
            hw.motors[2].setPower(-1);
            hw.motors[3].setPower(-1);
        }
        if(gamepad1.right_stick_y <0){
            hw.motors[0].setPower(-1);
            hw.motors[1].setPower(-1);
        }
        else {
            hw.motors[2].setPower(0);
            hw.motors[3].setPower(0);
            hw.motors[0].setPower(0);
            hw.motors[1].setPower(0);
        }



    }
    @Override
    public void stop(){

    }
}

