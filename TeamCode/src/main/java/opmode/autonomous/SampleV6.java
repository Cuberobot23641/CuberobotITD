/*
   this one is pretty straightforward
   start
   score first sample, extend intake
   drive to first sample
   intake first sample while retracting lift slides.
   after intake, transfer
   after transfer, lift up
   after lifting up, move back
   after moving back, score
   after scoring, drive to second samp + extend intake
   (btw it is now possible to move depo arm at the same time as lift since we wait for everything to finish)
   intake second sample while retracting lift slides
   after intake, transfer
   after transfer, lift up
   after lifting up, move back
   after moving back, score
   after scoring, drive to third samp + extend intake
   intake third sample while retracting lift slides
   after intake, transfer
   after transfer, lift up
   after lifting up, move back
   after moving back, score
   after scoring, drive to sub. add a parametric callback at 0.1 to retract lift
    */


package opmode.autonomous;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import robot.robots.SampleRobot;
import util.Condition;
import util.FiniteStateMachine;
import util.PositionCalculator;
import util.State;
import vision.SigmaPythonDetector;

import static robot.RobotConstantsAuto.*;

@Autonomous(name = "0+7 sample COMP test 2")
public class SampleV6 extends OpMode {
    // going up: set depo to 0.5, lift up slides. (all same time)
    // scoring: set depo to 0.32, then release
    // going down: set depo to 0.85, list down slides. (all same time)
    public SampleRobot robot;
    private double[] positions1;
    private double[] positions2;
    private double[] positions3;
    private Follower follower;
    private int pathState;
    private SigmaPythonDetector detector;
    private Timer pathTimer;
    private final Pose startPose = new Pose(9, 114, Math.toRadians(0));
    private FiniteStateMachine autoRunner;
    private PathChain scorePreload, grabSample2, grabSample3, scoreSample3, toSubmersible1, fromSubmersible1, toSubmersible2, fromSubmersible2, toSubmersible3;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 114.000, Point.CARTESIAN),
                                new Point(18.456, 119.662, Point.CARTESIAN),
                                new Point(19.000, 133.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .setPathEndTValueConstraint(0.995)
                .build();

        grabSample2 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(19.000, 133.000, Point.CARTESIAN),
                                new Point(19.000, 136.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-15))
                .setPathEndTValueConstraint(0.995)
                .build();


        grabSample3 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(19.000, 136.000, Point.CARTESIAN),
                                new Point(19.000, 138.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .build();

        scoreSample3 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(19.000, 138.000, Point.CARTESIAN),
                                new Point(18.000, 134.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-25))
                .setPathEndTValueConstraint(0.99)
                .build();


        toSubmersible1 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(18.000, 134.000, Point.CARTESIAN),
                                new Point(68.000, 122.000, Point.CARTESIAN),
                                new Point(68.000, 96.000, Point.CARTESIAN)
                        )
                )
                // add parametric callback: set intake to drop off pos,
                // later parametric callback: retract slides
                // 0.1: set lift to transfer
                // 0.1: set slides to 500
                // 0.3: change turret and elbow positions
                // 0.5: retract extension
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_TRANSFER);
                    robot.extension.setTargetPos(500);
                })
                .addParametricCallback(0.3, () -> {
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.5, () -> robot.extension.setTargetPos(0))
                .setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(-90))
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
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_TRANSFER);
                    robot.extension.setTargetPos(500);
                })
                .addParametricCallback(0.3, () -> {
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.5, () -> robot.extension.setTargetPos(0))
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
                    robot.lift.setTargetPos(LIFT_TRANSFER);
                    robot.extension.setTargetPos(500);
                })
                .addParametricCallback(0.3, () -> {
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.5, () -> robot.extension.setTargetPos(0))
                .addParametricCallback(0.6, () -> robot.deposit.setElbowDepositPos(0.6))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
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
                if (!follower.isBusy() && robot.extendLift.isFinished() && robot.extendExtension.isFinished()) {
                    robot.scoreSample.start();
                    robot.grabSample.start();
                    setPathState(2);
                }
                break;
            case 2:
                if (robot.scoreSample.isFinished() && robot.grabSample.isFinished()) {
                    follower.followPath(grabSample2, true);
                    robot.retractLift.start();
                    robot.transfer.start();
                    setPathState(3);
                }
                break;
            case 3:
                if (robot.retractLift.isFinished() && robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    robot.setPositions(positions2);
                    robot.extendExtension.start();
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() && robot.extendLift.isFinished() && robot.extendExtension.isFinished()) {
                    robot.scoreSample.start();
                    robot.grabSample.start();
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.scoreSample.isFinished() && robot.grabSample.isFinished()) {
                    follower.followPath(grabSample3, true);
                    robot.retractLift.start();
                    robot.transfer.start();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && robot.retractLift.isFinished() && robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    robot.setPositions(positions3);
                    robot.extendExtension.start();
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && robot.extendLift.isFinished() && robot.extendExtension.isFinished()) {
                    robot.scoreSample.start();
                    robot.grabSample.start();
                    setPathState(8);
                }
                break;
            case 8:
                if (robot.scoreSample.isFinished() && robot.grabSample.isFinished()) {
                    follower.followPath(scoreSample3, true);
                    robot.retractLift.start();
                    robot.transfer.start();
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && robot.retractLift.isFinished() && robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy() && robot.extendLift.isFinished() && robot.extendExtension.isFinished()) {
                    robot.scoreSample.start();
                    setPathState(17);
                }
                break;
            case 17:
                if (robot.scoreSample.isFinished()) {
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
                    robot.transfer.start();
                    follower.followPath(fromSubmersible1, true);
                    setPathState(22);
                }
                break;
//            case 21:
//                if (robot.extension.getExtensionPos() >= 550) {
//                    // now we can transfer
//                    robot.transfer.start();
//                    setPathState(22);
//                }
//                break;
            case 22:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy() && robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    setPathState(24);
                }
                break;

            case 24:
                if (robot.scoreSample.isFinished()) {
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
//                    robot.extension.setTargetPos(500);
//                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DEFAULT);
//                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
                    robot.transfer.start();
                    follower.followPath(fromSubmersible2, true);
                    setPathState(29);
                }
                break;
//            case 28:
//                if (robot.extension.getExtensionPos() >= 450) {
//                    // now we can transfer
//                    robot.transfer.start();
//                    setPathState(29);
//                }
//                break;
            case 29:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    setPathState(30);
                }
                break;
            case 30:
                if (!follower.isBusy() && robot.extendLift.isFinished()) {
                    robot.scoreSample.start();
                    setPathState(31);
                }
                break;
            case 31:
                if (robot.scoreSample.isFinished()) {
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

        positions1 = PositionCalculator.getPositions(-0.5, 27, -20);
        positions2 = PositionCalculator.getPositions(-0.5, 21.5, -15);
        positions3 = PositionCalculator.getPositions(-3.2, 21, 0);

        buildPaths();

        autoRunner = new FiniteStateMachine(
                new State()
                        .then(() -> {
                            follower.followPath(scorePreload);
                            robot.extendLift.start();
                        }),
                new State()
                        .addCondition(new Condition(() -> !follower.isBusy()))
                        .addCondition(new Condition(() -> robot.extendLift.isFinished()))
                        .then(() -> robot.scoreSample.start())
        );

        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot = new SampleRobot(hardwareMap);
    }

    @Override
    public void init_loop() {
        robot.deposit.setElbowDepositPos(0.85);
        robot.lift.setTargetPos(300);
        robot.loop();
    }

    @Override
    public void start() {
        detector.on();
        setPathState(0);
    }
}
