package opmode.test;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import robot.robots.AutonomousRobot;
import robot.robots.AutonomousRobot;
import vision.SigmaPythonDetector;


@Autonomous(name = "limelight detector python move test")
public class LimelightMoveTest extends OpMode {
    private SigmaPythonDetector detector;
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
            robot.intake.openIntakeClaw();
//            double[] distances = detector.getDistances();
//            if (Math.abs(distances[0]) != 0 && Math.abs(distances[1]) != 0) {
//                double[] positions = detector.getPositions();
//                robot.setExtensionInches(positions[0]);
//                robot.setTurretAngle(positions[1]);
//                robot.setWristAngle(positions[2]);
//                robot.clickbaitGrab.start();
//            }
            double[] positions = detector.getPositions();
            robot.setExtensionInches(positions[0]);
            robot.setTurretAngle(positions[1]);
            robot.setWristAngle(positions[2]);
            robot.clickbaitGrab.start();
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
        double[] positions1 = detector.getPositions();
        double[] positions2 = detector.getPositions2();
        telemetry.addData("positions1x", positions1[0]);
        telemetry.addData("positions1y", positions1[1]);
        telemetry.addData("positions1t", positions1[2]);
        telemetry.addData("positions2x", positions2[0]);
        telemetry.addData("positions2y", positions2[1]);
        telemetry.addData("positions2t", positions2[2]);

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
    }

    @Override
    public void start() {
        detector.on();
    }
}
