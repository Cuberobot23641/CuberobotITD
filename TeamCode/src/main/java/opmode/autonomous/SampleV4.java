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

@Autonomous(name = "0+6 sample post-comp NEW LOBSteR", group="comp")
public class SampleV4 extends OpMode {
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
                                new Point(15.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        grabSample1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(15.000, 129.000, Point.CARTESIAN),
                                new Point(20.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .setPathEndTimeoutConstraint(100)
                .build();

        scoreSample1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(20.000, 126.000, Point.CARTESIAN),
                                new Point(15.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        grabSample2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(15.000, 129.000, Point.CARTESIAN),
                                new Point(20.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .setPathEndTimeoutConstraint(200)
                .build();

        scoreSample2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(20.000, 126.000, Point.CARTESIAN),
                                new Point(15.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(14.000, 135.000, Point.CARTESIAN),
                                new Point(22.000, 128.000, Point.CARTESIAN)
        // new Point(22.000, 16.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(28))
                .setPathEndTValueConstraint(0.995)
                .setPathEndTimeoutConstraint(200)
                .build();

        scoreSample3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(22.000, 128.000, Point.CARTESIAN),
                                new Point(15.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(-45))
                .build();


        toSubmersible3 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(15.000, 129.000, Point.CARTESIAN),
                                new Point(68.000, 120.000, Point.CARTESIAN),
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
                    follower.followPath(grabSample1, true);
                    setPathState(69);
                }
                break;
            case 69:
                if (!follower.isBusy()) {
                    robot.retractLift.start();
                    setPathState(123456);
                }
                break;
            case 123456:
                if (robot.retractLift.isFinished()) {
                    setPathState(3);
                }
                break;
            case 3:
                if (robot.extendExtension.isFinished()) {
                    robot.fastGrab.start();
                    setPathState(4);
                }
                break;
            case 4:
                if (robot.fastGrab.isFinished()) {
                    robot.transfer.start();
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    robot.setPositions(positions2);
                    // robot.extendExtension.start();
                    setPathState(42);
                }
                break;
            case 42:
                if (robot.extendLift.isFinished()) {
                    follower.followPath(scoreSample1, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    robot.scoreSample.start();
                    setPathState(7);
                }
                break;
                case 7:
                    if (robot.scoreSample.isFinished()) {
                        follower.followPath(grabSample2, true);
                        setPathState(694);
                    }
                    break;
            case 694:
                if (!follower.isBusy()) {
                    robot.retractLift.start();
                    setPathState(1234567);
                }
                break;
            case 1234567:
                if (robot.retractLift.isFinished()) {
                    setPathState(8);
                }
                break;
            case 8:
                if (robot.extendExtension.isFinished()) {
                    robot.fastGrab.start();
                    setPathState(9);
                }
                break;
            case 9:
                if (robot.fastGrab.isFinished()) {
                    robot.transfer.start();
                    setPathState(10);
                }
                break;
            case 10:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    robot.setPositions(positions3);
                    setPathState(11);
                }
                break;
            case 11:
                if (robot.extendLift.isFinished()) {
                    follower.followPath(scoreSample2, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    robot.scoreSample.start();
                    setPathState(13);
                }
                break;
            case 13:
                if (robot.scoreSample.isFinished()) {
                    follower.followPath(grabSample3, true);
                    setPathState(695);
                }
                break;
            case 695:
                if (!follower.isBusy()) {
                    robot.retractLift.start();
                    setPathState(12345678);
                }
                break;
            case 12345678:
                if (robot.retractLift.isFinished()) {
                    setPathState(14);
                }
                break;
            case 14:
                if (robot.extendExtension.isFinished()) {
                    robot.fastGrab.start();
                    setPathState(15);
                }
                break;
            case 15:
                if (robot.fastGrab.isFinished()) {
                    robot.transfer.start();
                    setPathState(16);
                }
                break;
            case 16:
                if (robot.transfer.isFinished()) {
                    robot.extendLift.start();
                    robot.setPositions(positions3);
                    setPathState(17);
                }
                break;
            case 17:
                if (robot.extendLift.isFinished()) {
                    follower.followPath(scoreSample3, true);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    robot.scoreSample.start();
                    setPathState(19);
                }
                break;
            case 19:
                if (robot.scoreSample.isFinished()) {
                    follower.followPath(toSubmersible3);
                    setPathState(20);
                }
                break;
            case 20:
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
        detector = new SigmaPythonDetector(hardwareMap, "red sample");

        positions1 = PositionCalculator.getPositions(-5, 21, 0);
        positions2 = PositionCalculator.getPositions(5, 20.5, 0);
        positions3 = PositionCalculator.getPositions(-1.5, 21, 28);

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
        robot.deposit.setElbowDepositPos(0.85);
        robot.lift.setTargetPos(300);
        robot.loop();
    }

    @Override
    public void start() {
        detector.on();
        buildPaths();
        setPathState(0);
    }
}
