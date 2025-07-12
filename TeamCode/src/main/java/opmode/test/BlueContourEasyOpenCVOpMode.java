package opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import vision.BlueContourPipeline;

@TeleOp
public class BlueContourEasyOpenCVOpMode extends LinearOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        // Get webcam
        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Create camera
        int camViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(camName, camViewId);

        // Set pipeline
        BlueContourPipeline pipeline = new BlueContourPipeline();
        webcam.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(webcam, 30); // 30 = FPS
        // FtcDashboard.getInstance().startCameraStream(webcam);

        // Open camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera open error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            for (BlueContourPipeline.TargetInfo target : pipeline.getTargets()) {
                telemetry.addData("Target", "Dist: %.1f, Angle: %.1f", target.distance, target.angle);
            }

            BlueContourPipeline.TargetInfo closest = pipeline.getClosestTarget();
            if (closest != null) {
                telemetry.addData("Closest Target", "Dist: %.1f, Angle: %.1f", closest.distance, closest.angle);
            }

            telemetry.update();
            telemetry.addLine("Running");
            telemetry.update();
            sleep(50);
        }

        webcam.stopStreaming();
    }
}

