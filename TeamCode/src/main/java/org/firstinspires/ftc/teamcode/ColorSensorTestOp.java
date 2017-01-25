package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/**
 * Created by phanau on 1/22/16.
 * Test hardware color sensor
 */

@Autonomous(name="Color Sensor Test", group ="Autonomous")
//@Disabled

public class ColorSensorTestOp extends OpMode {

    hardwareDeclare hw;

    public void init() {
        // get hardware gyro
        hw = new hardwareDeclare(this);
    }

    public void loop() {
        telemetry.addData("red: ", hw.mColorSensor.red());
        telemetry.addData("green: ", hw.mColorSensor.green());
        telemetry.addData("blue: ", hw.mColorSensor.blue());

    }

    public void stop() {}

}
