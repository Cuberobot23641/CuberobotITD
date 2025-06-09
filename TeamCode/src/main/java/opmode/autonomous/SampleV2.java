package opmode.autonomous;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "0+8 sample auto v2")
public class SampleV2 extends OpMode {
    public SampleRobot robot;
    private double[] positions1;
    private double[] positions2;
    private double[] positions3;
    private Follower follower;
    private int pathState;
    private SigmaPythonDetector detector;
    private Timer pathTimer;
    private final Pose startPose = new Pose(9, 114, Math.toRadians(0));
    private PathChain scorePreload, grabSample3, toSubmersible1, fromSubmersible1, toSubmersible2, fromSubmersible2, toSubmersible3, fromSubmersible3;
    // probably unnecessary
    public void buildPaths() {
        // TODO: set custom zpams
        scorePreload = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 114.000, Point.CARTESIAN),
                                new Point(15.008, 121.893, Point.CARTESIAN),
                                new Point(20.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-25))
                .setPathEndTValueConstraint(0.995)
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(20.000, 131.000, Point.CARTESIAN),
                                new Point(16.000, 138.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-5), Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .build();


        toSubmersible1 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(16.000, 138.000, Point.CARTESIAN),
                                new Point(64.000, 122.000, Point.CARTESIAN),
                                new Point(64.000, 96.000, Point.CARTESIAN)
                        )
                )
                // add parametric callback: set intake to drop off pos,
                // later parametric callback: retract slides
                .addParametricCallback(0.15, () -> {
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.4, () -> robot.extension.setTargetPos(0))
                .addParametricCallback(0.4, () -> robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SAMPLE_SCORE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .setZeroPowerAccelerationMultiplier(6)
                .build();

        fromSubmersible1 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(64.000, 96.000, Point.CARTESIAN),
                                new Point(64.000, 122.000, Point.CARTESIAN),
                                new Point(14.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .build();

        toSubmersible2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(14.000, 131.000, Point.CARTESIAN),
                                new Point(64.000, 122.000, Point.CARTESIAN),
                                new Point(64.000, 96.000, Point.CARTESIAN)
                        )
                )
                .addParametricCallback(0.15, () -> {
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.4, () -> robot.extension.setTargetPos(0))
                .addParametricCallback(0.4, () -> robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SAMPLE_SCORE))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .setZeroPowerAccelerationMultiplier(6)
                .build();

        fromSubmersible2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(64.000, 96.000, Point.CARTESIAN),
                                new Point(64.000, 122.000, Point.CARTESIAN),
                                new Point(14.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .build();

        toSubmersible3 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(14.000, 131.000, Point.CARTESIAN),
                                new Point(64.000, 122.000, Point.CARTESIAN),
                                new Point(64.000, 96.000, Point.CARTESIAN)
                        )
                )
                .addParametricCallback(0.15, () -> {
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.4, () -> robot.extension.setTargetPos(0))
                .addParametricCallback(0.4, () -> robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SAMPLE_SCORE))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .setZeroPowerAccelerationMultiplier(6)
                .build();

        fromSubmersible3 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(64.000, 96.000, Point.CARTESIAN),
                                new Point(64.000, 122.000, Point.CARTESIAN),
                                new Point(14.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                robot.extendLift.start();
                robot.setPositions(positions1);
                robot.extendExtension.start();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    setPathState(123);
                }
                break;
            case 123:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    robot.grabSample.start();
                    setPathState(2);
                }
                break;
            case 2:
                if (robot.scoreSample.isFinished()) {
                    robot.retractLift.start();
                    setPathState(3);
                }
                break;
            case 3:
                if (robot.grabSample.isFinished()) {
                    robot.transfer.start();
                    follower.holdPoint(new Point(16, 133), Math.toRadians(-5));
                    setPathState(4);
                }
                break;
            case 4:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    robot.setPositions(positions2);
                    robot.extendExtension.start();
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    robot.grabSample.start();
                    setPathState(6);
                }
                break;
            case 6:
                if (robot.scoreSample.isFinished()) {
                    robot.retractLift.start();
                    setPathState(7);
                }
                break;
            case 7:
                if (robot.grabSample.isFinished()) {
                    robot.transfer.start();
                    follower.followPath(grabSample3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    robot.setPositions(positions3);
                    robot.extendExtension.start();
                    setPathState(9);
                }
                break;
            case 9:
                if (robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    robot.grabSample.start();
                    setPathState(10);
                }
                break;
            case 10:
                if (robot.scoreSample.isFinished()) {
                    robot.retractLift.start();
                    setPathState(11);
                }
                break;
            case 11:
                if (robot.grabSample.isFinished()) {
                    robot.transfer.start();
                    // perhaps holdPoint to a better point?
                    setPathState(12);
                }
                break;
            case 12:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    setPathState(13);
                }
                break;
            case 13:
                if (robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    // ts is scuffed, extend extension here
                    robot.extension.setTargetPos(500);
                    // robot.retractExtension.start();
                    setPathState(14);
                }
                break;
            case 14:
                if (robot.scoreSample.isFinished()) {
                    robot.retractLift.start();
                    follower.followPath(toSubmersible1, false);
                    // add parametric callback: set intake to drop off pos,
                    // later parametric callback: retract slides
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    setPathState(16);
                }
                break;
            case 16:
                double[] distances1 = detector.getDistances();
                if (distances1[0] != 0 && distances1[1] != 0 && pathTimer.getElapsedTimeSeconds() > 1) {
                    double[] positions = detector.getPositions();
                    robot.setPositions(positions);
                    robot.fastGrab.start();
                    setPathState(69);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(17);
                }
                break;
            case 69:
                if (robot.fastGrab.isFinished()) {
                    // we need to extend slides before transfer, also put intake uprobot.extension.setTargetPos(500);
                    robot.extension.setTargetPos(500);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DEFAULT);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
                    // robot.transfer.start();
                    follower.followPath(fromSubmersible1, true);
                    setPathState(70);
                }
                break;
            case 70:
                if (robot.extension.getExtensionPos() >= 460) {
                    // now we can transfer
                    robot.transfer.start();
                    setPathState(17);
                }
                break;
            case 17:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy() && robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    // nonononono la polici tung tung tung sahur
                    robot.extension.setTargetPos(500);
                    setPathState(420);
                }
                break;
            case 420:
                if (robot.scoreSample.isFinished()) {
                    robot.retractLift.start();
                    follower.followPath(toSubmersible2, false);
                    // add parametric callback: set intake to drop off pos,
                    // later parametric callback: retract slides
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    setPathState(20);
                }
                break;
            case 20:
                double[] distances2 = detector.getDistances();
                if (distances2[0] != 0 && distances2[1] != 0 && pathTimer.getElapsedTimeSeconds() > 1) {
                    double[] positions = detector.getPositions();
                    robot.setPositions(positions);
                    robot.fastGrab.start();
                    setPathState(691);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(691);
                }
                break;
            case 691:
                if (robot.fastGrab.isFinished()) {
                    // we need to extend slides before transfer, also put intake uprobot.extension.setTargetPos(500);
                    robot.extension.setTargetPos(500);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DEFAULT);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
                    // robot.transfer.start();
                    follower.followPath(fromSubmersible2, true);
                    setPathState(21);
                }
                break;
            case 21:
                if (robot.extension.getExtensionPos() >= 460) {
                    // now we can transfer
                    robot.transfer.start();
                    setPathState(22);
                }
                break;
            case 22:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy() && robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    // nonononono la polici tung tung tung sahur
                    robot.extension.setTargetPos(500);
                    setPathState(422);
                }
                break;
            case 422:
                if (robot.scoreSample.isFinished()) {
                    robot.retractLift.start();
                    follower.followPath(toSubmersible3, false);
                    // add parametric callback: set intake to drop off pos,
                    // later parametric callback: retract slides
                    setPathState(24);
                }
                break;
            case 24:
                if (!follower.isBusy()) {
                    setPathState(25);
                }
                break;
            case 25:
                double[] distances3 = detector.getDistances();
                if (distances3[0] != 0 && distances3[1] != 0 && pathTimer.getElapsedTimeSeconds() > 1) {
                    double[] positions = detector.getPositions();
                    robot.setPositions(positions);
                    robot.fastGrab.start();
                    setPathState(692);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(692);
                }
                break;
            case 692:
                if (robot.fastGrab.isFinished()) {
                    // we need to extend slides before transfer, also put intake uprobot.extension.setTargetPos(500);
                    robot.extension.setTargetPos(500);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DEFAULT);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
                    // robot.transfer.start();
                    follower.followPath(fromSubmersible3, true);
                    setPathState(26);
                }
                break;
            case 26:
                if (robot.extension.getExtensionPos() >= 460) {
                    // now we can transfer
                    robot.transfer.start();
                    setPathState(27);
                }
                break;
            case 27:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    setPathState(28);
                }
                break;
            case 28:
                if (!follower.isBusy() && robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    // nonononono la polici tung tung tung sahur
                    setPathState(29);
                }
                break;
            case 29:
                if (robot.scoreSample.isFinished()) {
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

        positions1 = PositionCalculator.getPositions(0, 24, -25);
        positions2 = PositionCalculator.getPositions(1, 23, -5);
        positions3 = PositionCalculator.getPositions(-3, 24.5, 0);

        try {
            sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot = new SampleRobot(hardwareMap);
    }

    @Override
    public void init_loop() {
        robot.deposit.setElbowDepositPos(0.7);
    }

    @Override
    public void start() {
        detector.on();
        buildPaths();
        setPathState(0);
    }
}
