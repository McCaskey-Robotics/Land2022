package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.math.MathContext;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class ADriverControled extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo claw;
        CRServo lift;

        TelemetryPacket packet = new TelemetryPacket();

        FtcDashboard dashboard;

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        String VUFORIA_LICENSE_KEY = "AY2/Abz/////AAABmRiO7MLRC0KDvnef+RFVZBpppJ3+LKlP8nhpBndIeCprKByTKnhCOsWOit5Ode+yJPeTV6+YpcLkjkl2HYtIEaEJ3Uz4zjxfpJAXpUE//y/qg5XyFkvH+vIt1jdFi4exZ8JhS78iwq4G28aZRyf0H9P7lWOCPg6qaWNIAiY6w/LIRg0KvPM4FBYqftio4dTHs6NXpvq7rNe91r5Y/Q35bqzONSElj9n8jvxic7AdOe9Z8eG/bXwUnyjqPFsRV+tTmI5tQ1obW7Q356NmRvKEI0V29G2NoqCsejorhWk8TKRRl71ipz2x/pT98HIHe32Pz8IgkPfQI249ykGeUl5Uoc8hxLChMMUY9cTF9Dq/5BFD";

        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        double clawPosition = 0.5;
        double liftPosition = 0.5;

        claw = hardwareMap.get(Servo.class,"claw");
        lift = hardwareMap.get(CRServo.class,"lift");

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y / 2,
                            -gamepad1.left_stick_x / 2,
                            -gamepad1.right_trigger / 5 + gamepad1.left_trigger / 5
                    )
            );
            drive.update();


            if (gamepad1.dpad_up) {
                clawPosition += 0.01;
            }
            if (gamepad1.dpad_down) {
                clawPosition -= 0.01;
            }

            clawPosition = Math.min(1,clawPosition);
            clawPosition = Math.max(0,clawPosition);

            claw.setPosition(clawPosition);
            lift.setPower(gamepad1.right_stick_x);

            packet.put("gamepad", gamepad1);

            packet.put("type", gamepad1.type());

            dashboard.sendTelemetryPacket(packet);

        }
    }
}
