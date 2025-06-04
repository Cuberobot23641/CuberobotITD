package opmode.autonomous;


import static com.pedropathing.util.Constants.setConstants;
import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import static robot.RobotConstants.*;

import pedroPathing.constants.FConstants;
import robot.robots.AutonomousRobot;
import util.PositionCalculator;
import vision.SigmaPythonDetector;

@Autonomous(name = "7+0 grab fling start")
public class SpecBlueV7 extends OpMode {
    public AutonomousRobot robot;
    private Follower follower;
    private int pathState;
    private Timer pathTimer;
    private Timer totalTimer;
    private double totalElapsed = 0;
    private SigmaPythonDetector detector;
    private final Pose startPose = new Pose(9, 66, Math.toRadians(0));
    private final Pose grabPose = new Pose(9, 34, Math.toRadians(0));
    private final Pose scorePreloadPose = new Pose(43, 66, Math.toRadians(0));
    private PathChain scorePreload, grabSample12, grabSample3, grabSpec1, scoreSpec1, grabSpec2, scoreSpec2, grabSpec3, scoreSpec3, grabSpec4, scoreSpec4, grabSpec5, scoreSpec5, grabSpec6, scoreSpec6;
    private double[] positions1;
    private double[] positions2;
    private double[] positions3;
    public void buildPaths() {
        // TODO: set custom zpams
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreloadPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0.1, () -> robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF))
                .build();

        grabSample12 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(43.000, 66.000, Point.CARTESIAN),
                                new Point(20.000, 19.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .setPathEndTimeoutConstraint(200)
                .addParametricCallback(0.5, () -> {
                    robot.setPositions(positions1);
                    robot.extendExtension.start();
                })
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(20.000, 19.00, Point.CARTESIAN),
                                new Point(30.000, 18.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .setPathEndTimeoutConstraint(200)
                .setPathEndTValueConstraint(0.995)
                .build();

        grabSpec1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(30.000, 18.000, Point.CARTESIAN),
                                new Point(9.000, 32.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        scoreSpec1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(9.000, 32.000, Point.CARTESIAN),
                                new Point(43.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabSpec2 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(43.000, 72.000, Point.CARTESIAN),
                                new Point(9.000, 32.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpec2 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(9.000, 32.000, Point.CARTESIAN),
                                new Point(43.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabSpec3 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(43.000, 72.000, Point.CARTESIAN),
                                new Point(9.000, 32.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpec3 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(9.000, 32.000, Point.CARTESIAN),
                                new Point(43.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabSpec4 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(43.000, 72.000, Point.CARTESIAN),
                                new Point(9.000, 32.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpec4 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(9.000, 32.000, Point.CARTESIAN),
                                new Point(43.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabSpec5 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(43.000, 72.000, Point.CARTESIAN),
                                new Point(9.000, 32.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpec5 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(9.000, 32.000, Point.CARTESIAN),
                                new Point(43.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabSpec6 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(43.000, 72.000, Point.CARTESIAN),
                                new Point(9.000, 32.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpec6 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(9.000, 32.000, Point.CARTESIAN),
                                new Point(43.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, false);
                robot.scoreSpecimen.start();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && robot.scoreSpecimen.isFinished()) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(2);
                }
                break;
            case 2:
                if (robot.prepareGrabSpecimen.getIndex() > 0) {
                    follower.setMaxPower(0.8);
                    follower.followPath(grabSample12, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    robot.grabSample.start();
                    setPathState(4);
                }
                break;
            case 4:
                if (robot.grabSample.isFinished()) {
                    robot.fastRetract.start();
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.fastRetract.isFinished()) {
                    robot.setPositions(positions2);
                    robot.fastGrab.start();
                    setPathState(6);
                }
                break;
            case 6:
                if (robot.fastGrab.isFinished()) {
                    robot.fastRetract.start();
                    follower.followPath(grabSample3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (robot.fastRetract.isFinished() && !follower.isBusy()) {
                    robot.setPositions(positions3);
                    robot.fastGrab.start();
                    setPathState(8);
                }
                break;
            case 8:
                if (robot.fastGrab.isFinished()) {
                    robot.fastRetract.start();
                    follower.setMaxPower(1);
                    follower.followPath(grabSpec1, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(10);
                }
                break;
            case 10:
                if (robot.grabSpecimen.isFinished()) {
                    follower.followPath(scoreSpec1, true);
                    robot.scoreSpecimen.start();
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && robot.scoreSpecimen.isFinished()) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(69);
                }
                break;
            case 69:
                double[] distances1 = detector.getDistances();
                // shouldnt be necessary
                if (distances1[0] != 0 && distances1[1] != 0 && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    double[] positions = detector.getPositions();
                    robot.setPositions(positions);
                    robot.fastGrab.start();
                    setPathState(12);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(12);
                }
                break;
            case 12:
                if (robot.fastGrab.isFinished() && robot.prepareGrabSpecimen.isFinished()) {
                    robot.retractExtension.start();
                    // resetting
                    robot.setExtensionInches(0);
                    robot.setTurretAngle(0);
                    robot.setWristAngle(0);
                    follower.followPath(grabSpec2, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (robot.retractExtension.isFinished() && !follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(14);
                }
                break;
            case 14:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec2, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy() && robot.scoreSpecimen.isFinished()) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(430);
                }
                break;
            case 430:
                double[] distances2 = detector.getDistances();
                // shouldnt be necessary
                if (distances2[0] != 0 && distances2[1] != 0 && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    double[] positions = detector.getPositions();
                    robot.setPositions(positions);
                    robot.fastGrab.start();
                    setPathState(16);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(16);
                }
                break;
            case 16:
                if (robot.fastGrab.isFinished() && robot.prepareGrabSpecimen.isFinished()) {
                    robot.retractExtension.start();
                    // maybe this will save power?
                    detector.off();
                    // YOOOOOOO
                    // follower.setDrivePIDF();
                    follower.followPath(grabSpec3, true);
                    setPathState(17);
                }
                break;
            case 17:
                if (robot.retractExtension.isFinished() && !follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(18);
                }
                break;
            case 18:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec3, true);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(20);
                }
                break;
            case 20:
                if (robot.prepareGrabSpecimen.getIndex() > 0) {
                    follower.followPath(grabSpec4, true);
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(22);
                }
                break;
            case 22:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec4, true);
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(24);
                }
                break;
            case 24:
                if (robot.prepareGrabSpecimen.getIndex() > 0) {
                    follower.followPath(grabSpec5, true);
                    setPathState(25);
                }
                break;
            case 25:
                if (!follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(26);
                }
                break;
            case 26:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec5, true);
                    setPathState(27);
                }
                break;
            case 27:
                if (!follower.isBusy()) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(28);
                }
                break;
            case 28:
                if (robot.prepareGrabSpecimen.getIndex() > 0) {
                    follower.followPath(grabSpec6, true);
                    setPathState(29);
                }
                break;
            case 29:
                if (!follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(30);
                }
                break;
            case 30:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec4, true);
                    setPathState(31);
                }
                break;
            case 31:
                if (!follower.isBusy()) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(32);
                }
                break;
            case 32:
                if (robot.prepareGrabSpecimen.getIndex() > 0) {
                    totalElapsed = totalTimer.getElapsedTimeSeconds();
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        robot.loop();
        detector.update();
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("total elapsed time", totalTimer.getElapsedTimeSeconds());
        telemetry.addData("final elapsed time", totalElapsed);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        totalTimer = new Timer();
        setConstants(LConstants.class, FConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        positions1 = PositionCalculator.getPositions(-5, 20, 0);
        positions2 = PositionCalculator.getPositions(5, 20, 0);
        positions3 = PositionCalculator.getPositions(0, 15, -45);
        detector = new SigmaPythonDetector(hardwareMap, "blue sample");
        try {
            sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot = new AutonomousRobot(hardwareMap);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        totalTimer.resetTimer();
        detector.on();
        buildPaths();
        setPathState(0);
    }
}
