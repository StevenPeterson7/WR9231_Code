package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_FTC2017;

@Autonomous(name="StraightRedAutonomous", group ="Autonomous")
//@Disabled
public class AutonomousRedStraight extends AutonomousRedMain {


        public void init() {
            super.init(true);
        }


}
