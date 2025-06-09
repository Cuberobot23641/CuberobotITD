package opmode.autonomous;


import static com.pedropathing.util.Constants.setConstants;
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
import static robot.RobotConstantsAuto.*;

import robot.robots.AutonomousRobot;

@Autonomous(name = "7+0 grab rotate start")
public class SpecBlueV5 extends OpMode {
    public AutonomousRobot robot;
    private Follower follower;
    private int pathState;
    private Timer pathTimer;
    private final Pose startPose = new Pose(9, 66, Math.toRadians(0));
    private final Pose grabPose = new Pose(9, 34, Math.toRadians(0));
    private final Pose scorePreloadPose = new Pose(44, 66, Math.toRadians(0));
    private PathChain scorePreload, grabSample1, moveSample1, grabSample2, moveSample2, grabSample3, grabSpec1;
    public void buildPaths() {
        // TODO: set custom zpams
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreloadPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0.1, () -> robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF))
                .build();

        grabSample1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(44.000, 66.000, Point.CARTESIAN),
                                new Point(27.000, 67.000, Point.CARTESIAN),
                                new Point(29.000, 44.000, Point.CARTESIAN)
                        )
                )
//                .setCustomHeadingInterpolation(t -> (1-Math.exp(-t)) * ((2*Math.E) / (1+Math.E)) * Math.toRadians(-45))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .addParametricCallback(0.6, () -> robot.extension.setTargetInches(11.3))
                .addParametricCallback(0.6, () -> robot.intake.setTurretPos(INTAKE_TURRET_DEFAULT))
                .addParametricCallback(0.6, () -> robot.intake.setWristAngle(-45))
                .addParametricCallback(0.7, () -> robot.intake.setElbowIntakePos(INTAKE_ELBOW_HOVER))
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                .setPathEndTValueConstraint(0.99)
                .setZeroPowerAccelerationMultiplier(10)
                .build();
        moveSample1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(29.000, 44.000, Point.CARTESIAN),
                                new Point(29.000, 44.000, Point.CARTESIAN)
                        )
                )
                .addParametricCallback(0.85, () -> robot.intake.openIntakeClaw())
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-110))
                .setPathEndTValueConstraint(0.95)
                .setZeroPowerAccelerationMultiplier(10)
                .build();
        // TODO: test this, it could be more accurate with p2p
//        align = new PathBuilder()
//                .addPath(new BezierPoint(new Point(31, 72-distX*1.03)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
        grabSample2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(29.000, 44.000, Point.CARTESIAN),
                                new Point(29.000, 31.000, Point.CARTESIAN)
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(-110), Math.toRadians(-45))
                .setPathEndTValueConstraint(0.99)
                .setZeroPowerAccelerationMultiplier(10)
                .build();
        moveSample2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(29.000, 31.000, Point.CARTESIAN),
                                new Point(29.000, 26.000, Point.CARTESIAN)
                        )
                )
                .addParametricCallback(0.85, () -> robot.intake.openIntakeClaw())
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-110))
                .setPathEndTValueConstraint(0.95)
                .setZeroPowerAccelerationMultiplier(10)
                .build();
        grabSample3 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(29.000, 26.000, Point.CARTESIAN),
                                new Point(29.000, 21.000, Point.CARTESIAN)
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(-110), Math.toRadians(-45))
                .setPathEndTValueConstraint(0.99)
                .setZeroPowerAccelerationMultiplier(10)
                .build();
        grabSpec1 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(29.000, 21.000, Point.CARTESIAN),
                                new Point(29.000, 34.000, Point.CARTESIAN),
                                new Point(9.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
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
                    follower.followPath(grabSample1, true);
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
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_HOVER);
                    follower.followPath(moveSample1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabSample2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    robot.grabSample.start();
                    setPathState(7);
                }
                break;
            case 7:
                if (robot.grabSample.isFinished()) {
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_HOVER);
                    follower.followPath(moveSample2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(grabSample3, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    robot.grabSample.start();
                    setPathState(10);
                }
                break;
            case 10:
                if (robot.grabSample.isFinished()) {
                    robot.fastRetract.start();
                    follower.setMaxPower(1.0);
                    follower.followPath(grabSpec1, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            // TODO: the very boring and repeated process of scoring can be done in a loop for 5 times
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        robot.loop();
        // detector.update();
        follower.update();
        autonomousPathUpdate();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        setConstants(LConstants.class, FConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        // detector = new LimelightDetector(hardwareMap, "blue sample");
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
        buildPaths();
        setPathState(0);
    }
}
