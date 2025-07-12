package opmode.test;


import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.android.Utils;
import org.opencv.core.Mat;


import vision.BlueContourProcessor;

@TeleOp(name = "BlueContour Dashboard Stream", group = "Vision")
public class RedContourVisionProcessorOpmode extends LinearOpMode {

    private VisionPortal visionPortal;
    private BlueContourProcessor processor;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        processor = new BlueContourProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // change if needed
                .addProcessor(processor)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(false) // disable internal view since we're using Dashboard
                .build();

        // visionPortal.getCameraControl()setRotation(CameraRotation.SIDEWAYS_LEFT);

        // Setup Dashboard
        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(visionPortal, 0); // 0 ms delay = realtime

        telemetry.addLine("Dashboard stream started. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            BlueContourProcessor.TargetInfo closest = processor.getClosestTarget();
            int count = processor.getTargetCount();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Count", count);

            if (closest != null) {
                packet.put("Closest Target xDistance (mm)", closest.xDistance);
                packet.put("Closest Target yDistance (mm)", closest.yDistance);
                packet.put("Closest Target Angle (deg)", closest.angle);
            } else {
                packet.put("Closest Target", "None");
            }

            dashboard.sendTelemetryPacket(packet);
            sleep(50);
        }

        visionPortal.close();
    }
}

