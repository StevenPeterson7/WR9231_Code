package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.pow;

@TeleOp(name="servoTest", group="TeleOp")
public class testTeleOP extends OpMode{

    hardwareDeclare hw;
    private float motorPower = 1.f;
    double lPos=0.5;
    double rPos=0.5;
    double lTwistPos=0;
    double rTwistPos=0;

    @Override
    public void init(){

        // Get our hardware
        hw = new hardwareDeclare(this);
        hw.glyphLiftArms[0].setPosition(lPos);
        hw.glyphLiftArms[1].setPosition(rPos);
        hw.armTwist[0].setPosition(0);
        hw.armTwist[1].setPosition(0);
        hw.ColorSensor.enableLed(false);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        hw.imu.initialize(parameters);


    }

    @Override
    public void loop(){



       /* telemetry.addData("servo1 position:", hw.glyphLiftArms[0].getPosition());
        telemetry.addData("servo2 position:", hw.glyphLiftArms[1].getPosition());

        //
         telemetry.addData("gyro: ", hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);


        //if the arm positions are not within set bounds, reset them
        if(lPos<0){
            lPos=0;
        }
        if(rPos<0){
            rPos=0;
        }
        if(lPos>1){
                lPos=1;
        }
        if(rPos>1){
                rPos=1;
        }
        //slowly close the arm when the bumper is pushed
        if(gamepad2.right_bumper){
            rPos-=0.07;
        }
        if(gamepad2.left_bumper){
            lPos-=0.07;
        }

        /*
        when the left button is pressed, move the block to the left but
        make sure that there is a constant pressure on the block
         */
       /* if(gamepad2.dpad_left){
            if(rPos<1) {
                rPos += 0.11;
            }
            if(lPos>0) {
                lPos -= 0.07;
            }
        }*/
        /*
        do the opposite for the right side
         */
       /* if(gamepad2.dpad_right){
            if(lPos<1) {
                lPos += 0.11;
            }
            if(rPos>0) {
                rPos -= 0.07;
            }
        }

        //slowly open the arms when the trigger is pulled with the rate depending on how much the triggers are pulled
        lPos+=gamepad2.left_trigger/15;
        rPos+=gamepad2.right_trigger/15;

        //apply the new positions
        hw.glyphLiftArms[0].setPosition(lPos);
        hw.glyphLiftArms[1].setPosition(rPos);

        telemetry.addData("right trigger", gamepad2.right_trigger);*/

        if(gamepad2.a){
            lTwistPos=1;
            rTwistPos=1;
        }else if (gamepad2.b){
            lTwistPos=0;
            rTwistPos=0;
        }

        if(lTwistPos<0){
            lTwistPos=0;
        }
        if(rTwistPos<0){
            rTwistPos=0;
        }
        if(lTwistPos>1){
            lTwistPos=1;
        }
        if(rTwistPos>1){
            rTwistPos=1;
        }
        rTwistPos+=gamepad2.right_stick_y/50;
        lTwistPos+=gamepad2.left_stick_y/50;


        hw.armTwist[0].setPosition(lTwistPos);
        hw.armTwist[1].setPosition(rTwistPos);
        telemetry.addData("left twist", lTwistPos);
        telemetry.addData("right twist", rTwistPos);


    }
    @Override
    public void stop(){
    }
}

