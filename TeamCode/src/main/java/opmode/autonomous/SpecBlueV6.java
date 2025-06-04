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

@Autonomous(name = "aspirity ahh auto")
public class SpecBlueV6 extends OpMode {
    public AutonomousRobot robot;
    private Follower follower;
    private int pathState;
    private Timer pathTimer;
    private final Pose startPose = new Pose(9, 66, Math.toRadians(0));
    private final Pose grabPose = new Pose(9, 34, Math.toRadians(0));
    private final Pose scorePreloadPose = new Pose(43, 66, Math.toRadians(0));
    private double[] positions1;
    private double[] positions2;
    private double[] positions3;
    private PathChain scorePreload, grabSample1, moveSample1, grabSample2, moveSample2, grabSample3, grabSpec1;
    public void buildPaths() {
        // TODO: MAKE PATHS LOL
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreloadPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0.1, () -> robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF))
                .build();

        grabSample1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(43.000, 66.000, Point.CARTESIAN),
                                new Point(20.000, 23.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .addParametricCallback(0.5, () -> {
                    robot.setPositions(positions1);
                    robot.extendExtension.start();
                })
                .build();

        grabSample2 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(20.000, 23.000, Point.CARTESIAN),
                                new Point(20.000, 13.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(20.000, 13.00, Point.CARTESIAN),
                                new Point(25.000, 10.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20))
                .setPathEndTValueConstraint(0.995)
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
                    robot.fastRetract.start();
                    follower.followPath(grabSample2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.fastRetract.isFinished()) {
                    robot.setPositions(positions2);
                    robot.extendExtension.start();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && robot.extendExtension.isFinished()) {
                    robot.grabSample.start();
                    setPathState(7);
                }
                break;
            case 7:
                if (robot.grabSample.isFinished()) {
                    robot.fastRetract.start();
                    follower.followPath(grabSample3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (robot.fastRetract.isFinished()) {
                    robot.setPositions(positions3);
                    robot.extendExtension.start();
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && robot.extendExtension.isFinished()) {
                    robot.grabSample.start();
                    setPathState(10);
                }
                break;
            case 10:
                if (robot.grabSample.isFinished()) {
                    robot.fastRetract.start();
                    setPathState(11);
                }
                break;
            case 11:
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
        positions1 = PositionCalculator.getPositions(0, 21, 0);
        positions2 = PositionCalculator.getPositions(0, 21, 0);
        positions3 = PositionCalculator.getPositions(-0.53, 16.37, 0);
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
