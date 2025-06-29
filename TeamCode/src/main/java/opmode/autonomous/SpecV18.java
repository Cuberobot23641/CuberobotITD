package opmode.autonomous;


import static com.pedropathing.util.Constants.setConstants;
import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import static robot.RobotConstantsAuto.*;

import robot.RobotConstantsTeleOp;
import robot.robots.AutonomousRobot;
import util.PositionCalculator;
import vision.SigmaPythonDetector;

@Autonomous(name = "7+0 safety+tangential")
public class SpecV18 extends OpMode {
    public AutonomousRobot robot;
    private Follower follower;
    private int pathState;
    private Timer pathTimer;
    private Timer totalTimer;
    private double totalElapsed = 0;
    private SigmaPythonDetector detector;
    private final Pose startPose = new Pose(9, 66, Math.toRadians(0));
    private final Pose grabPose = new Pose(9, 34, Math.toRadians(0));
    private final Pose scorePreloadPose = new Pose(40, 66, Math.toRadians(0));
    private PathChain scorePreload, grabSample12, grabSample3, grabSpec1, scoreSpec1, grabSpec2, scoreSpec2, grabSpec3, scoreSpec3, grabSpec4, scoreSpec4, grabSpec5, scoreSpec5, grabSpec6, scoreSpec6;
    private double[] positions1;
    private double[] positions2;
    private double[] positions3;

    private double[] subGrab2Distances;
    private double prevX;
    private double prevY;

    public void buildPaths() {
        // TODO: CHANGE ALL PATHS
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreloadPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(4)
                .setPathEndTValueConstraint(0.95)
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0.1, () -> robot.lift.setTargetPos(RobotConstantsTeleOp.LIFT_SPEC_SCORE))
                .addParametricCallback(0.1, () -> robot.deposit.setElbowDepositPos(RobotConstantsTeleOp.DEPOSIT_ELBOW_SPEC_SCORE))
                .build();

        grabSample12 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(40.000, 66.000, Point.CARTESIAN),
                                new Point(28.394, 65.713, Point.CARTESIAN),
                                new Point(20.000, 18.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .addParametricCallback(0.5, () -> {
                    robot.setPositions(positions1);
                    robot.extendExtension.start();
                })
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_SPEC_GRAB);;
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                })
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(20.000, 18.00, Point.CARTESIAN),
                                new Point(22.000, 16.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-25))
                .setPathEndTValueConstraint(0.995)
                .build();

        grabSpec1 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(22.000, 16.000, Point.CARTESIAN),
                                new Point(23.324, 34.073, Point.CARTESIAN),
                                new Point(18.659, 33.262, Point.CARTESIAN),
                                new Point(9.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setPathEndTValueConstraint(0.93)
                .addParametricCallback(0.9, () -> {
                    robot.intake.openIntakeClaw();
                })
                .setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .build();

        scoreSpec1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 34.000, Point.CARTESIAN),
                                new Point(38.000, 74.000, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(0)
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.7, () -> follower.setMaxPower(0.7))
                .addPath(
                        new BezierPoint(new Point(42, 78, Point.CARTESIAN))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabSpec2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(42.000, 78.000, Point.CARTESIAN),
                                new Point(15.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_SPEC_GRAB);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                })
                .addParametricCallback(0.55, () -> {
                    robot.intake.setWristPos(INTAKE_WRIST_DROP_OFF);
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.9, () -> {
                    robot.intake.openIntakeClaw();
                })
                .setPathEndTimeoutConstraint(0)
                .setReversed(true)
                .addPath(
                        new BezierPoint(new Point(9, 34, Point.CARTESIAN))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpec2 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 34.000, Point.CARTESIAN),
                                new Point(38.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(0)
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.7, () -> follower.setMaxPower(0.7))
                .addPath(
                        new BezierPoint(new Point(42, 76, Point.CARTESIAN))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabSpec3 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(42.000, 76.000, Point.CARTESIAN),
                                new Point(15.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_SPEC_GRAB);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                })
                .addParametricCallback(0.55, () -> {
                    robot.intake.setWristPos(INTAKE_WRIST_DROP_OFF);
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.9, () -> {
                    robot.intake.openIntakeClaw();
                })
                .setPathEndTimeoutConstraint(0)
                .setReversed(true)
                .addPath(
                        new BezierPoint(new Point(9, 34, Point.CARTESIAN))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpec3 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 34.000, Point.CARTESIAN),
                                new Point(38.000, 70.000, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(0)
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.7, () -> follower.setMaxPower(0.7))
                .addPath(
                        new BezierPoint(new Point(42, 74, Point.CARTESIAN))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabSpec4 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(42.000, 74.000, Point.CARTESIAN),
                                new Point(15.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_SPEC_GRAB);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                })
                .setPathEndTimeoutConstraint(0)
                .setReversed(true)
                .addPath(
                        new BezierPoint(new Point(9, 34, Point.CARTESIAN))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpec4 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 34.000, Point.CARTESIAN),
                                new Point(38.000, 68.000, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(0)
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.7, () -> follower.setMaxPower(0.7))
                .addPath(
                        new BezierPoint(new Point(42, 72, Point.CARTESIAN))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabSpec5 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(42.000, 72.000, Point.CARTESIAN),
                                new Point(15.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_SPEC_GRAB);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                })
                .setPathEndTimeoutConstraint(0)
                .setReversed(true)
                .addPath(
                        new BezierPoint(new Point(9, 34, Point.CARTESIAN))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpec5 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 34.000, Point.CARTESIAN),
                                new Point(38.000, 66.000, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(0)
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.7, () -> follower.setMaxPower(0.7))
                .addPath(
                        new BezierPoint(new Point(42, 70, Point.CARTESIAN))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabSpec6 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(42.000, 70.000, Point.CARTESIAN),
                                new Point(15.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_SPEC_GRAB);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                })
                .setPathEndTimeoutConstraint(0)
                .setReversed(true)
                .addPath(
                        new BezierPoint(new Point(9, 34, Point.CARTESIAN))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpec6 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 34.000, Point.CARTESIAN),
                                new Point(38.000, 64.000, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(0)
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.7, () -> follower.setMaxPower(0.7))
                .addPath(
                        new BezierPoint(new Point(42, 68, Point.CARTESIAN))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, false);
                // robot.scoreSpecimen.start();
                setPathState(1);
                break;
            case 1:
//                if (follower.getCurrentTValue() > 0.85 && follower.getAcceleration().getXComponent() < -100) {
//                    robot.prepareGrabSpecimen.start();
//                    setPathState(2);
//                }
                if (follower.atPose(new Pose(40, 66, Math.toRadians(0)), 2, 2, 0.02)) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(2);
                }
//                if (!follower.isBusy() && robot.scoreSpecimen.isFinished()) {
//                    robot.prepareGrabSpecimen.start();
//                    setPathState(2);
//                }
                break;
            case 2:
                if (robot.prepareGrabSpecimen.isFinished()) {
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
                    robot.firstSample.start();
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.firstSample.isFinished()) {
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
                    robot.fastGrab2.start();
                    setPathState(8);
                }
                break;
            case 8:
                if (robot.fastGrab2.isFinished()) {
                    robot.thirdSample.start();
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
                    follower.followPath(scoreSpec1, false);
                    robot.scoreSpecimen.start();
                    setPathState(11);
                }
                break;
            case 11:
                if (follower.atPose(new Pose(42, 78, Math.toRadians(0)), 2, 2, 0.02)) {
                    follower.setMaxPower(1);
                    robot.prepareGrabSpecimen.start();
                    setPathState(69);
                }
                break;
            case 69:
                double[] distances1 = detector.getDistances();
                if (distances1[0] != 0 && distances1[1] != 0 && pathTimer.getElapsedTimeSeconds() > 0.7) {
                    robot.setPositions(detector.getPositions());
                    robot.subGrab.start();
                    setPathState(12);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(12);
                }
                break;
            case 12:
                if (robot.subGrab.isFinished() && robot.prepareGrabSpecimen.isFinished()) {
                    robot.retractSub.start();
                    robot.setExtensionInches(0);
                    robot.setTurretAngle(0);
                    robot.setWristAngle(0);
                    follower.setMaxPower(1);
                    follower.followPath(grabSpec2);
                    setPathState(13);
                }
                break;
            case 13:
                if (follower.atPose(new Pose(9, 34, Math.toRadians(0)), 2, 2, 0.02)) {
                    robot.grabSpecimen.start();
                    setPathState(14);
                }
                break;
            case 14:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec2);
                    setPathState(15);
                }
                break;
            case 15:
                if (follower.atPose(new Pose(42, 76, Math.toRadians(0)), 2, 2, 0.02)) {
                    follower.setMaxPower(1);
                    robot.prepareGrabSpecimen.start();
                    setPathState(440);
                }
                break;
            case 440:
                double[] distances2 = detector.getDistances();
                if (distances2[0] != 0 && distances2[1] != 0 && pathTimer.getElapsedTimeSeconds() > 0.7) {
                    robot.setPositions(detector.getPositions());
                    robot.subGrab.start();
                    setPathState(16);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(16);
                }
                break;
            case 16:
                if (robot.subGrab.isFinished() && robot.prepareGrabSpecimen.isFinished()) {
                    robot.retractSub.start();
                    detector.off();
                    follower.setMaxPower(1);
                    follower.followPath(grabSpec3);
                    setPathState(17);
                }
                break;
            case 17:
                if (follower.atPose(new Pose(9, 34, Math.toRadians(0)), 2, 2, 0.02)) {
                    robot.grabSpecimen.start();
                    setPathState(18);
                }
                break;
            case 18:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec3, false);
                    setPathState(19);
                }
                break;
            case 19:
                if (follower.atPose(new Pose(42, 74, Math.toRadians(0)), 2, 2, 0.02)) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(20);
                }
                break;
            case 20:
                if (robot.prepareGrabSpecimen.isFinished()) {
                    follower.setMaxPower(1);
                    follower.followPath(grabSpec4);
                    setPathState(21);
                }
                break;
            case 21:
                if (follower.atPose(new Pose(9, 34, Math.toRadians(0)), 2, 2, 0.02)) {
                    robot.grabSpecimen.start();
                    setPathState(22);
                }
                break;
            case 22:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec4, false);
                    setPathState(23);
                }
                break;
            case 23:
                if (follower.atPose(new Pose(42, 72, Math.toRadians(0)), 2, 2, 0.02)) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(24);
                }
                break;
            case 24:
                if (robot.prepareGrabSpecimen.isFinished()) {
                    follower.setMaxPower(1);
                    follower.followPath(grabSpec5);
                    setPathState(25);
                }
                break;
            case 25:
                if (follower.atPose(new Pose(9, 34, Math.toRadians(0)), 2, 2, 0.02)) {
                    robot.grabSpecimen.start();
                    setPathState(26);
                }
                break;
            case 26:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec5, false);
                    setPathState(27);
                }
                break;
            case 27:
                if (follower.atPose(new Pose(42, 70, Math.toRadians(0)), 2, 2, 0.02)) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(28);
                }
                break;
            case 28:
                if (robot.prepareGrabSpecimen.isFinished()) {
                    follower.setMaxPower(1);
                    follower.followPath(grabSpec6);
                    setPathState(29);
                }
                break;
            case 29:
                if (follower.atPose(new Pose(9, 34, Math.toRadians(0)), 2, 2, 0.02)) {
                    robot.grabSpecimen.start();
                    setPathState(30);
                }
                break;
            case 30:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec6, false);
                    setPathState(31);
                }
                break;
            case 31:
                if (follower.atPose(new Pose(42, 70, Math.toRadians(0)), 2, 2, 0.02)) {
                    totalElapsed = totalTimer.getElapsedTimeSeconds();
                    robot.scoreSpecimen.start();
                    setPathState(32);
                }
                break;
            case 32:
                if (robot.scoreSpecimen.isFinished()) {
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
        telemetry.addData("acceleration x", follower.getAcceleration().getXComponent());
        System.out.println("x pos: " + follower.getPose().getX());
        System.out.println("y pos: " + follower.getPose().getY());
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        totalTimer = new Timer();
        setConstants(LConstants.class, FConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        positions1 = PositionCalculator.getPositions(-4.8, 22, 0);
        positions2 = PositionCalculator.getPositions(5, 20.5, 0);
        positions3 = PositionCalculator.getPositions(0.8, 21.5, -25);
        buildPaths();
        detector = new SigmaPythonDetector(hardwareMap, "blue sample");
        try {
            sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot = new AutonomousRobot(hardwareMap);
        //robot.lift.setTwoMotors(true);
    }

    @Override
    public void init_loop() {
        robot.deposit.setElbowDepositPos(0.85);
        robot.lift.setTargetPos(500);
        robot.loop();
        follower.update();
    }

    @Override
    public void start() {
        totalTimer.resetTimer();
        detector.on();
        setPathState(0);
    }
}
