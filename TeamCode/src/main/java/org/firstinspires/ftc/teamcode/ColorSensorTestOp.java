package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/**
 * Created by phanau on 1/22/16.
 * Test hardware color sensor
 */

@Autonomous(name="Test: MR Color Sensor Test 1", group ="Test")
@Disabled
public class ColorSensorTestOp extends OpMode {

    private ModernRoboticsI2cColorSensor mColorSensor;

    public ColorSensorTestOp() {
    }

    public void init() {
        // get hardware gyro
        mColorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("cs");
        mColorSensor.enableLed(false);
    }

    public void loop() {
        telemetry.addData("red: ", mColorSensor.red());
        telemetry.addData("green: ", mColorSensor.green());
        telemetry.addData("blue: ", mColorSensor.blue());

    }

    public void stop() {}

}
