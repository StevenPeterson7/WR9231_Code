package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "VuforiaTest")
public class VuforiaTest extends LinearOpMode
{
    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String VUFORIA_KEY = "AUKJRQz/////AAAAGfz0WwzKYEnatzjleo3MKWkMdv6RctxiFJ2/sDJCZZqdFzSwi/uL/D0fEu8/KW84cGYiDojFtELyRwTOcZKUuLnp/M5KWJocaO/7GFOmpUnhusDc5kieOSKigNXbbnjs1NcQj/b8tXU1BTq+iaZl3UWnbtuVphKG0kPN1XitzndEtUSw7+MIETx+s4B014JYLGNZEeKVdiwd2dlnJQIYpnIhDh7kQVP/MboYlWZxanSQT3FlwzxoNBr/TFZHLoPIDT0bc1B9C2X3TapiDJgakYkdPwAobXAMdHiCYzyCAea9pZdzsFBZxFKpvkLJjQueFV3ApAK0XeGh+7jtVLCI5/XxVCO3zF3HtvitsAVUpCYF";

    public void runOpMode() throws InterruptedException
    {
        setupVuforia();

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        visionTargets.activate();

        while(opModeIsActive())
        {
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            telemetry.update();
            idle();
        }
    }

    public void setupVuforia()
    {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");

        target = visionTargets.get(0);
        target.setName("Gears Target");
        target.setLocation(createMatrix(0, 500, 0, 90, 0, 90));

        phoneLocation = createMatrix(0, 255, 0, 90, 0, 0);

        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}
