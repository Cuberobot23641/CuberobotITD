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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.ImuCommands;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import robot.robots.AutonomousRobot;
import vision.SigmaPythonDetector;

import static robot.RobotConstants.*;
@Autonomous(name = "7+0 fling")
public class SpecBlueV1 extends OpMode {
    // TODO: idea: when grabbing specs, check if robot velocity suddenly decreases and then grab

    public AutonomousRobot robot;
    private Follower follower;
    private int pathState;
    private SigmaPythonDetector detector;
    private Timer pathTimer;
    private double[] pos2;
    private final Pose startPose = new Pose(9, 72, Math.toRadians(0));
    private final Pose scorePreloadPose = new Pose(36, 72, Math.toRadians(0));
    private PathChain push, scorePreload, grabSpec1, scoreSpec1, grabSpec2, scoreSpec2, grabSample12, grabSample3;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreloadPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0.1, () -> robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF))
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0.5, () -> follower.setMaxPower(0.6))
                .setPathEndTValueConstraint(0.95)
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        grabSpec1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(36.000, 72.000, Point.CARTESIAN),
                                new Point(9.000, 32.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0.3, () -> {
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                    robot.lift.setTargetPos(0);
                })
                .setPathEndTimeoutConstraint(0)
                .setZeroPowerAccelerationMultiplier(4)
                .setPathEndTValueConstraint(0.9)
                .build();

        scoreSpec1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(9.000, 32.000, Point.CARTESIAN),
                                new Point(36.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0.5, () -> follower.setMaxPower(0.6))
                .setPathEndTValueConstraint(0.95)
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        grabSpec2 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(36.000, 72.000, Point.CARTESIAN),
                                new Point(9.000, 32.000, Point.CARTESIAN)
                        )
                )
                .addParametricCallback(0.3, () -> {
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                    robot.lift.setTargetPos(0);
                })
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .setZeroPowerAccelerationMultiplier(4)
                .setPathEndTValueConstraint(0.9)
                .build();

        scoreSpec2 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(9.000, 32.000, Point.CARTESIAN),
                                new Point(36.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .setZeroPowerAccelerationMultiplier(4)
                .setPathEndTValueConstraint(0.95)
                .build();

        grabSample12 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(36.000, 72.000, Point.CARTESIAN),
                                new Point(24.000, 18.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                // creative thing? SEE IF IT WORKS
                .addParametricCallback(0.7, () -> follower.setMaxPower(0.5))
                .addParametricCallback(0.3, () -> {
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                })
                .setPathEndTValueConstraint(0.99)
                .setPathEndTimeoutConstraint(100)
                .setZeroPowerAccelerationMultiplier(8)
                .setPathEndVelocityConstraint(0.1)
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(22.000, 18.00, Point.CARTESIAN),
                                new Point(24.000, 14.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-10))
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.99)
                .setZeroPowerAccelerationMultiplier(10)
                .setPathEndVelocityConstraint(0.1)
                .build();

    }
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(scorePreload, true); // we either put as true or holdpoint somewhere else
                robot.scoreSpecimen.start();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && robot.scoreSpecimen.isFinished()) {
                    robot.prepareGrabSpecimen.start();
//                    //robot.setPositions(detector.getPositions());
//                    robot.fastGrab.start();
                    setPathState(69);
                }
                break;
            case 69:
                double[] distances = detector.getDistances();
                if (Math.abs(distances[0]) < 8 && Math.abs(distances[1]) < 30 && pathTimer.getElapsedTimeSeconds() > 0.8) {
                    double[] positions = detector.getPositions();
                    robot.setExtensionInches(positions[0]);
                    robot.setTurretAngle(positions[1]);
                    robot.setWristAngle(positions[2]);
                    robot.fastGrab.start();
                    setPathState(2);
                }

                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(2);
                }
                break;
            case 2:
                if (robot.fastGrab.isFinished() && robot.prepareGrabSpecimen.isFinished()) {
                    robot.retractExtension.start();
                    // resetting
                    robot.setExtensionInches(0);
                    robot.setTurretAngle(0);
                    robot.setWristAngle(0);
                    follower.setMaxPower(1);
                    follower.followPath(grabSpec1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (robot.retractExtension.isFinished() && !follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(4);
                }
                break;
            case 4:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && robot.scoreSpecimen.isFinished()) {
                    robot.prepareGrabSpecimen.start();
//                    //robot.setPositions(detector.getPositions());
//                    robot.fastGrab.start();
                    setPathState(420);
                }
                break;
            case 420:
                double[] distances2 = detector.getDistances();
                if (Math.abs(distances2[0]) < 8 && Math.abs(distances2[1]) < 30 && pathTimer.getElapsedTimeSeconds() > 0.8) {
                    double[] positions = detector.getPositions();
                    robot.setExtensionInches(positions[0]);
                    robot.setTurretAngle(positions[1]);
                    robot.setWristAngle(positions[2]);
                    robot.fastGrab.start();
                    setPathState(6);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(6);
                }
                break;
            case 6:
                if (robot.fastGrab.isFinished() && robot.prepareGrabSpecimen.isFinished()) {
                    robot.retractExtension.start();
                    robot.setExtensionInches(0);
                    robot.setTurretAngle(0);
                    robot.setWristAngle(0);
                    follower.setMaxPower(1);
                    follower.followPath(grabSpec2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (robot.retractExtension.isFinished() && !follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(8);
                }
                break;
            case 8:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    // false:
                    follower.followPath(scoreSpec2, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && robot.scoreSpecimen.isFinished() && follower.getVelocityMagnitude() < 0.1) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(10);
                }
                break;
            case 10:
                // being safe here, not .getIndex() > 0
                if (robot.prepareGrabSpecimen.isFinished()) {
                    follower.followPath(grabSample12);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12:
                double[] distances3 = detector.getDistances();
                if (Math.abs(distances3[0]) < 8 && Math.abs(distances3[1]) < 30) {
                    // idk why
                    double[] pos2 = detector.getPositions2();
                    double[] positions = detector.getPositions();
                    robot.setExtensionInches(positions[0]);
                    robot.setTurretAngle(positions[1]);
                    robot.setWristAngle(positions[2]);
                    robot.fastGrab.start();
                    setPathState(13);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(13);
                }
                break;
            case 13:
                if (robot.fastGrab.isFinished()) {
                    robot.fastRetract.start();
                    robot.setExtensionInches(0);
                    robot.setTurretAngle(0);
                    robot.setWristAngle(0);
                    setPathState(14);
                }
                break;
            case 14:
                // && pathTimer.getElapsedTimeSeconds() > 1
                if (robot.fastRetract.isFinished()) {
                    robot.setPositions(pos2);
                    robot.fastGrab.start();
                    setPathState(15);
                }
                break;
            case 15:
                if (robot.fastGrab.isFinished()) {
                    robot.fastRetract.start();
                    follower.setMaxPower(0.6);
                    follower.followPath(grabSample3, true);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 0.1 && detector.isSampleThere()) {
                    robot.setPositions(detector.getPositions());
                    robot.fastGrab.start();
                    setPathState(18);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(18);
                }
                break;
            case 18:
                if (robot.fastGrab.isFinished()) {
                    robot.fastRetract.start();
                    setPathState(19);
                }
                break;
            case 19:
                if (robot.fastRetract.isFinished()) {
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
        detector.update();
        robot.loop();
        follower.update();
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
        autonomousPathUpdate();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        // setConstants(LConstants.class, FConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
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
        detector.on();
        buildPaths();
        setPathState(0);
    }
}

