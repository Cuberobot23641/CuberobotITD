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
import robot.robots.AutonomousRobot;
import util.PositionCalculator;
import vision.SigmaPythonDetector;

import static robot.RobotConstants.*;

import java.util.List;

@Autonomous(name = "0+5 sample auto vision")
public class SampleV1 extends OpMode {
    public AutonomousRobot robot;
    private double[] positions1;
    private double[] positions2;
    private double[] positions3;
    private Follower follower;
    private int pathState;
    private SigmaPythonDetector detector;
    private Timer pathTimer;
    private final Pose startPose = new Pose(9, 114, Math.toRadians(0));
    private PathChain scorePreload, grabSample1, scoreSample1, grabSample2, scoreSample2, grabSample3, scoreSample3, toSubmersible, fromSubmersible;
    // probably unnecessary
    public void buildPaths() {
        // TODO: set custom zpams
        scorePreload = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 114.000, Point.CARTESIAN),
                                new Point(15.617, 120.879, Point.CARTESIAN),
                                new Point(12.000, 132.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .setPathEndTValueConstraint(0.95)
                .build();

        grabSample1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(12.000, 132.000, Point.CARTESIAN),
                                new Point(19.000, 121.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .setPathEndTimeoutConstraint(100)
                .setZeroPowerAccelerationMultiplier(4)
                .setPathEndVelocityConstraint(0.1)
                .build();

        scoreSample1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(19.000, 121.000, Point.CARTESIAN),
                                new Point(12.000, 132.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .setPathEndTValueConstraint(0.95)
                .setPathEndTimeoutConstraint(100)
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        grabSample2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(12.000, 132.000, Point.CARTESIAN),
                                new Point(19.000, 132.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.99)
                .setZeroPowerAccelerationMultiplier(4)
                .setPathEndVelocityConstraint(0.1)
                .build();

        scoreSample2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(19.000, 132.000, Point.CARTESIAN),
                                new Point(12.000, 132.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .setPathEndTValueConstraint(0.95)
                .setPathEndTimeoutConstraint(100)
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(12.000, 132.000, Point.CARTESIAN),
                                new Point(30.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(45))
                .setPathEndTValueConstraint(0.99)
                .setPathEndTimeoutConstraint(100)
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        scoreSample3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(30.000, 126.000, Point.CARTESIAN),
                                new Point(12.000, 132.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-45))
                .setPathEndTValueConstraint(0.95)
                .setPathEndTimeoutConstraint(100)
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        toSubmersible = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(12.000, 132.000, Point.CARTESIAN),
                                new Point(60.237, 124.124, Point.CARTESIAN),
                                new Point(66.000, 98.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .setPathEndTValueConstraint(0.95)
                .setPathEndTimeoutConstraint(100)
                .setZeroPowerAccelerationMultiplier(4)
                .setPathEndVelocityConstraint(0.1)
                .build();

        fromSubmersible = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(66.000, 98.000, Point.CARTESIAN),
                                new Point(60.237, 124.124, Point.CARTESIAN),
                                new Point(12.000, 132.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .setPathEndTValueConstraint(0.99)
                .setPathEndTimeoutConstraint(100)
                .setZeroPowerAccelerationMultiplier(6)
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                robot.transferUp.start();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && robot.transferUp.isFinished()) {
                    robot.transferDown.start();
                    setPathState(2);
                }
                break;
            case 2:
                if (robot.transferDown.getIndex() > 0) {
                    follower.setMaxPower(0.8);
                    follower.followPath(grabSample1, true);
                    robot.setPositions(positions1);
                    robot.extendExtension.start();
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
                    robot.transferIn.start();
                    follower.followPath(scoreSample1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.transferIn.isFinished()) {
                    robot.transferUp.start();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && robot.transferUp.isFinished()) {
                    robot.transferDown.start();
                    setPathState(7);
                }
                break;
            case 7:
                if (robot.transferDown.getIndex() > 0) {
                    follower.followPath(grabSample2, true);
                    robot.setPositions(positions2);
                    robot.extendExtension.start();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    robot.grabSample.start();
                    setPathState(9);
                }
                break;
            case 9:
                if (robot.grabSample.isFinished()) {
                    robot.transferIn.start();
                    follower.followPath(scoreSample2, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (robot.transferIn.isFinished()) {
                    robot.transferUp.start();
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && robot.transferUp.isFinished()) {
                    robot.transferDown.start();
                    setPathState(12);
                }
                break;
            case 12:
                if (robot.transferDown.getIndex() > 0) {
                    follower.followPath(grabSample3, true);
                    robot.setPositions(positions3);
                    robot.extendExtension.start();
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    robot.grabSample.start();
                    setPathState(14);
                }
                break;
            case 14:
                if (robot.grabSample.isFinished()) {
                    robot.transferIn.start();
                    follower.followPath(scoreSample3, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (robot.transferIn.isFinished()) {
                    robot.transferUp.start();
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy() && robot.transferUp.isFinished()) {
                    robot.transferDown.start();
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

        positions1 = PositionCalculator.getPositions(0, 20, 0);
        positions2 = PositionCalculator.getPositions(0, 20, 0);
        positions3 = PositionCalculator.getPositions(0, 15, -45);

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

