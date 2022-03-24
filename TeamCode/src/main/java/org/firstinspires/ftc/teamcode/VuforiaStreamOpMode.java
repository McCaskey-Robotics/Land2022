package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class VuforiaStreamOpMode extends LinearOpMode {
    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "AY2/Abz/////AAABmRiO7MLRC0KDvnef+RFVZBpppJ3+LKlP8nhpBndIeCprKByTKnhCOsWOit5Ode+yJPeTV6+YpcLkjkl2HYtIEaEJ3Uz4zjxfpJAXpUE//y/qg5XyFkvH+vIt1jdFi4exZ8JhS78iwq4G28aZRyf0H9P7lWOCPg6qaWNIAiY6w/LIRg0KvPM4FBYqftio4dTHs6NXpvq7rNe91r5Y/Q35bqzONSElj9n8jvxic7AdOe9Z8eG/bXwUnyjqPFsRV+tTmI5tQ1obW7Q356NmRvKEI0V29G2NoqCsejorhWk8TKRRl71ipz2x/pT98HIHe32Pz8IgkPfQI249ykGeUl5Uoc8hxLChMMUY9cTF9Dq/5BFD";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}