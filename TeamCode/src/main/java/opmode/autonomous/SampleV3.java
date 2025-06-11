package opmode.autonomous;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import robot.robots.SampleRobot;
import util.PositionCalculator;
import vision.SigmaPythonDetector;

import static robot.RobotConstantsAuto.*;

@Autonomous(name = "0+7 sample comp auto")
public class SampleV3 extends OpMode {
    public SampleRobot robot;
    private double[] positions1;
    private double[] positions2;
    private double[] positions3;
    private Follower follower;
    private int pathState;
    private SigmaPythonDetector detector;
    private Timer pathTimer;
    private final Pose startPose = new Pose(9, 114, Math.toRadians(0));
    private PathChain scorePreload, grabSample1, scoreSample1, grabSample2, scoreSample2, grabSample3, scoreSample3, toSubmersible1, fromSubmersible1, toSubmersible2, fromSubmersible2, toSubmersible3, fromSubmersible3;
    // probably unnecessary
    public void buildPaths() {
        // TODO: set custom zpams
        scorePreload = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 114.000, Point.CARTESIAN),
                                new Point(14.806, 121.285, Point.CARTESIAN),
                                new Point(15.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();

        grabSample1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(15.000, 131.000, Point.CARTESIAN),
                                new Point(19.000, 128.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-15))
                .setPathEndTValueConstraint(0.995)
                .setPathEndTimeoutConstraint(100)
                .build();

        scoreSample1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(19.000, 128.000, Point.CARTESIAN),
                                new Point(15.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(-30))
                .build();

        grabSample2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(14.000, 135.000, Point.CARTESIAN),
                                new Point(19.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .setPathEndTimeoutConstraint(200)
                .build();

        scoreSample2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(19.000, 131.000, Point.CARTESIAN),
                                new Point(15.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(14.000, 135.000, Point.CARTESIAN),
                                new Point(19.000, 138.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .setPathEndTimeoutConstraint(200)
                .build();

        scoreSample3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(19.000, 138.000, Point.CARTESIAN),
                                new Point(14.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();


        toSubmersible1 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(14.000, 130.000, Point.CARTESIAN),
                                new Point(68.000, 122.000, Point.CARTESIAN),
                                new Point(68.000, 96.000, Point.CARTESIAN)
                        )
                )
                // add parametric callback: set intake to drop off pos,
                // later parametric callback: retract slides
                .addParametricCallback(0.1, () -> {
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.3, () -> robot.extension.setTargetPos(0))
                .addParametricCallback(0.3, () -> robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SAMPLE_SCORE))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        fromSubmersible1 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(68.000, 96.000, Point.CARTESIAN),
                                new Point(68.000, 122.000, Point.CARTESIAN),
                                new Point(13.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .addParametricCallback(0.75, () -> follower.setMaxPower(0.7))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        toSubmersible2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(13.000, 131.000, Point.CARTESIAN),
                                new Point(68.000, 122.000, Point.CARTESIAN),
                                new Point(68.000, 96.000, Point.CARTESIAN)
                        )
                )
                // add parametric callback: set intake to drop off pos,
                // later parametric callback: retract slides
                .addParametricCallback(0.1, () -> {
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.3, () -> robot.extension.setTargetPos(0))
                .addParametricCallback(0.3, () -> robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SAMPLE_SCORE))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        fromSubmersible2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(68.000, 96.000, Point.CARTESIAN),
                                new Point(68.000, 122.000, Point.CARTESIAN),
                                new Point(13.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .addParametricCallback(0.75, () -> follower.setMaxPower(0.7))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        toSubmersible3 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(13.000, 131.000, Point.CARTESIAN),
                                new Point(68.000, 122.000, Point.CARTESIAN),
                                new Point(68.000, 96.000, Point.CARTESIAN)
                        )
                )
                // add parametric callback: set intake to drop off pos,
                // later parametric callback: retract slides
                .addParametricCallback(0.1, () -> {
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.3, () -> robot.extension.setTargetPos(0))
                .addParametricCallback(0.3, () -> robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SAMPLE_SCORE))
                .addParametricCallback(0.6, () -> robot.deposit.setElbowDepositPos(0.6))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        fromSubmersible3 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(68.000, 96.000, Point.CARTESIAN),
                                new Point(68.000, 122.000, Point.CARTESIAN),
                                new Point(14.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(4)
                .build();



    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.7);
                follower.followPath(scorePreload, true);
                robot.extendLift.start();
                robot.setPositions(positions1);
                robot.extendExtension.start();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    setPathState(2);
                }
                break;
            case 2:
                if (robot.scoreSample.isFinished()) {
                    FollowerConstants.holdPointHeadingScaling = 0.5;
                    FollowerConstants.holdPointTranslationalScaling = 0.5;
                    follower.followPath(grabSample1, true);
                    setPathState(69);
                }
            case 69:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    robot.retractLift.start();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && robot.extendExtension.isFinished()) {
                    robot.grabSample.start();
                    setPathState(4);
                }
                break;
            case 4:
                if (robot.grabSample.isFinished()) {
                    robot.transfer.start();
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    robot.setPositions(positions2);
                    robot.extendExtension.start();
                    setPathState(42);
                }
                break;
            case 42:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    follower.followPath(scoreSample1, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    setPathState(7);
                }
                break;
            case 7:
                if (robot.scoreSample.isFinished()) {
                    follower.followPath(grabSample2, true);
                    setPathState(691);
                }
                break;
            case 691:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.retractLift.start();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && robot.extendExtension.isFinished()) {
                    robot.grabSample.start();
                    setPathState(9);
                }
                break;
            case 9:
                if (robot.grabSample.isFinished()) {
                    robot.transfer.start();
                    setPathState(10);
                }
                break;
            case 10:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    robot.setPositions(positions3);
                    robot.extendExtension.start();
                    setPathState(421);
                }
                break;
            case 421:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(scoreSample2, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    setPathState(12);
                }
                break;
            case 12:
                if (robot.scoreSample.isFinished()) {
                    follower.followPath(grabSample3, true);
                    setPathState(692);
                }
                break;
            case 692:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.retractLift.start();
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy() && robot.extendExtension.isFinished()) {
                    robot.grabSample.start();
                    setPathState(14);
                }
                break;
            case 14:
                if (robot.grabSample.isFinished()) {
                    robot.transfer.start();
                    setPathState(15);
                }
                break;
            case 15:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    setPathState(422);
                }
                break;
            case 422:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(scoreSample3, true);
                    setPathState(16);
                }
                break;
            case 16:
                if (robot.extendLift.isFinished() && !follower.isBusy()) {
                    robot.scoreSample.start();
                    robot.extension.setTargetPos(500);
                    setPathState(123);
                }
                break;
            case 123:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(17);
                }
                break;
            case 17:
                if (robot.scoreSample.isFinished()) {
                    robot.retractLift.start();
                    follower.setMaxPower(1);
                    follower.followPath(toSubmersible1, false);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    setPathState(19);
                }
                break;
            case 19:
                double[] distances1 = detector.getDistances();
                if (distances1[0] != 0 && distances1[1] != 0 && pathTimer.getElapsedTimeSeconds() > 1) {
                    double[] positions = detector.getPositions();
                    robot.setPositions(positions);
                    robot.fastGrab.start();
                    setPathState(20);
                }
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(20);
                }
                break;
            case 20:
                if (robot.fastGrab.isFinished()) {
                    // we need to extend slides before transfer, also put intake uprobot.extension.setTargetPos(500);
                    robot.extension.setTargetPos(500);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DEFAULT);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
                    follower.followPath(fromSubmersible1, true);
                    setPathState(21);
                }
                break;
            case 21:
                if (robot.extension.getExtensionPos() >= 450) {
                    // now we can transfer
                    robot.transfer2.start();
                    setPathState(22);
                }
                break;
            case 22:
                if (robot.transfer2.isFinished()) {
                    robot.extendLift.start();
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy() && robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    robot.extension.setTargetPos(500);
                    setPathState(456);
                }
                break;
            case 456:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(24);
                }
                break;
            case 24:
                if (robot.scoreSample.isFinished()) {
                    robot.retractLift.start();
                    follower.setMaxPower(1);
                    follower.followPath(toSubmersible2, false);
                    setPathState(25);
                }
                break;
            case 25:
                if (!follower.isBusy()) {
                    setPathState(26);
                }
                break;
            case 26:
                double[] distances2 = detector.getDistances();
                if (distances2[0] != 0 && distances2[1] != 0 && pathTimer.getElapsedTimeSeconds() > 1) {
                    double[] positions = detector.getPositions();
                    robot.setPositions(positions);
                    robot.fastGrab.start();
                    setPathState(27);
                }
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(27);
                }
                break;
            case 27:
                if (robot.fastGrab.isFinished()) {
                    // we need to extend slides before transfer, also put intake uprobot.extension.setTargetPos(500);
                    robot.extension.setTargetPos(500);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DEFAULT);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
                    follower.followPath(fromSubmersible2, true);
                    setPathState(28);
                }
                break;
            case 28:
                if (robot.extension.getExtensionPos() >= 450) {
                    // now we can transfer
                    robot.transfer2.start();
                    setPathState(29);
                }
                break;
            case 29:
                if (robot.transfer2.isFinished()) {
                    robot.extendLift.start();
                    setPathState(30);
                }
                break;
            case 30:
                if (!follower.isBusy() && robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    robot.extension.setTargetPos(500);
                    setPathState(31);
                }
                break;
            case 31:
                if (robot.scoreSample.isFinished()) {
                    robot.lift.setTargetPos(0);
                    follower.setMaxPower(1);
                    follower.followPath(toSubmersible3, false);
                    setPathState(32);
                }
                break;
            case 32:
                if (!follower.isBusy()) {
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
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        detector = new SigmaPythonDetector(hardwareMap, "yellow sample");

        positions1 = PositionCalculator.getPositions(-0.5, 21.5, -15);
        positions2 = PositionCalculator.getPositions(0, 20, 0);
        positions3 = PositionCalculator.getPositions(-3, 20.5, 0);

        try {
            sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot = new SampleRobot(hardwareMap);
        // robot.lift.setTwoMotors(true);
    }

    @Override
    public void init_loop() {
        robot.deposit.setElbowDepositPos(0.7);
        robot.lift.setTargetPos(500);
        robot.loop();
    }

    @Override
    public void start() {
        detector.on();
        buildPaths();
        setPathState(0);
    }
}
