package opmode.test;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import robot.robots.AutonomousRobot;
import robot.robots.AutonomousRobot;
import vision.SigmaPythonDetector;


@Autonomous(name = "limelight detector blue move test")
public class LimelightBlueTest extends OpMode {
    private SigmaPythonDetector detector;
    private double loopTime = 0;
    private Timer loopTimer;
    private AutonomousRobot robot;
    private Gamepad gp1;
    private Gamepad cgp1;
    private Gamepad pgp1;

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        pgp1.copy(cgp1);
        cgp1.copy(gp1);

        if (gamepad1.a && !pgp1.a) {
            double[] distances1 = detector.getDistances();
            // set to 0.1 to be fast
            robot.intake.openIntakeClaw();
            // TODO: check smaller times now that we have faster loop times
            if (distances1[0] != 0 && distances1[1] != 0) {
                double[] positions = detector.getPositions();
                robot.setPositions(positions);
                robot.fastGrab.start();
            }
        }

        if (gamepad1.b && !pgp1.b) {
            robot.retractExtension.start();
            robot.setExtensionInches(0);
            robot.setTurretAngle(0);
            robot.setWristAngle(0);
        }



//        telemetry.addData("y distance", positions[0]);
//        telemetry.addData("turret angle", positions[1]);
//        telemetry.addData("wrist angle", positions[2]);
        detector.update();
        robot.loop();
        loopTime = loopTimer.getElapsedTimeSeconds();
        loopTimer.resetTimer();
        double[] positions1 = detector.getPositions();
        double[] positions2 = detector.getPositions2();
        telemetry.addData("positions1x", positions1[0]);
        telemetry.addData("positions1y", positions1[1]);
        telemetry.addData("positions1t", positions1[2]);
        telemetry.addData("positions2x", positions2[0]);
        telemetry.addData("positions2y", positions2[1]);
        telemetry.addData("positions2t", positions2[2]);

        telemetry.addData("loop time", loopTime);

        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        detector = new SigmaPythonDetector(hardwareMap, "blue sample");
        robot = new AutonomousRobot(hardwareMap);
        gp1 = gamepad1;
        cgp1 = new Gamepad();
        pgp1 = new Gamepad();
        loopTimer = new Timer();
    }

    @Override
    public void start() {
        detector.on();
    }
}
