package opmode.autonomous;

import static com.pedropathing.util.Constants.setConstants;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

import pedroPathing.constants.LConstants;
import pedroPathing.constants.FConstants;

import robot.robots.AutonomousRobot;

@Autonomous(name = "Pedro 6+1")
public class OldPathing extends OpMode {
    GoBildaPinpointDriver odo;
    public AutonomousRobot robot;
    private Follower follower;

    private Timer pathTimer;
    private int pathState;

    private final Pose startPose = new Pose(8, 72, Math.toRadians(0));
    private final Pose grabPose = new Pose(7, 30, Math.toRadians(180));
    private final Pose scorePose = new Pose(37.5, 72, Math.toRadians(0));

    private final Pose score1Pose = new Pose(35, 70, Math.toRadians(180));
    private final Pose score2Pose = new Pose(35, 68, Math.toRadians(180));
    private final Pose score3Pose = new Pose(35, 66, Math.toRadians(180));
    private final Pose score4Pose = new Pose(35, 64.5, Math.toRadians(180));
    private final Pose score5Pose = new Pose(35, 63, Math.toRadians(180));
    private final Pose score6Pose = new Pose(36, 62, Math.toRadians(180));

    private final Pose sample1Pose = new Pose(15, 40, Math.toRadians(225));
    private final Pose sampleScorePose = new Pose(12, 134, Math.toRadians(315));
    private PathChain scorePreload, grabSpec1, scoreSpec1, push, scoreSpec2, grabSpec3, scoreSpec3, grabSpec4, scoreSpec4, grabSpec5, scoreSpec5, grabSpec6, scoreSpec6, grabSample1, scoreSample1, pushFinal;
    public void buildPaths() {
        // TODO: set custom zpams
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabSpec1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(scorePose),
                                new Point(5, 30, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), grabPose.getHeading())
                .setPathEndTimeoutConstraint(0.99)
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        scoreSpec1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(5, 30, Point.CARTESIAN),
                                new Point(score1Pose)
                        )
                )
                .setPathEndTimeoutConstraint(0.9)
                .setPathEndVelocityConstraint(1) // in/s?
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        push = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(40.000, 76.000, Point.CARTESIAN),
                                new Point(20, 40, Point.CARTESIAN),
                                new Point(59.733, 41.778, Point.CARTESIAN),
                                new Point(48.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))

                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(48.000, 25.000, Point.CARTESIAN),
                                new Point(20, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(4)
                .setPathEndVelocityConstraint(40) // in/s?
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(20, 25.000, Point.CARTESIAN),
                                new Point(41, 24.889, Point.CARTESIAN),
                                new Point(40.000, 14.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(6)
                .setPathEndVelocityConstraint(40) // in/s?
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(41.000, 15.000, Point.CARTESIAN),
                                new Point(26.000, 14.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(4)
                .setPathEndVelocityConstraint(40) // in/s?
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(26.000, 14.000, Point.CARTESIAN),
                                new Point(45, 10, Point.CARTESIAN),
                                new Point(43.000, 7.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(6)
                .setPathEndVelocityConstraint(40) // in/s?
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(43.000, 7.000, Point.CARTESIAN),
                                new Point(26, 7, Point.CARTESIAN)
                        )
                )
                .setZeroPowerAccelerationMultiplier(4)
                .setPathEndVelocityConstraint(20) // in/s?
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(26, 7, Point.CARTESIAN),
                                new Point(26, 30),
                                new Point(grabPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                // increased zpam so that it doesnt slwo down at end
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndTimeoutConstraint(0.99)
                .setPathEndVelocityConstraint(3) // in/s?
                //.addParametricCallback(0.4, () -> robot.getArmController().setTargetExtensionPos(0))
                .build();

        pushFinal = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(43.000, 8.000, Point.CARTESIAN),
                                new Point(8.000, 8.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndVelocityConstraint(10) // in/s?
                .build();

        scoreSpec2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(score2Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.9)
                .setPathEndVelocityConstraint(1) // in/s?
                .setZeroPowerAccelerationMultiplier(5)
                .build();


        grabSpec3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(score2Pose),
                                new Point(grabPose)
                        )
                )
                .setPathEndTimeoutConstraint(0.99)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        scoreSpec3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(score3Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndVelocityConstraint(1) // in/s?
                .setPathEndTimeoutConstraint(0.9)
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        grabSpec4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(score3Pose),
                                new Point(grabPose)
                        )
                )
                .setPathEndTimeoutConstraint(0.99)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
        scoreSpec4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(score4Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.9)
                .setPathEndVelocityConstraint(1) // in/s?
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        grabSpec5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(score4Pose),
                                new Point(grabPose)
                        )
                )
                .setPathEndTimeoutConstraint(0.99)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        scoreSpec5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(score5Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.9)
                .setPathEndVelocityConstraint(1) // in/s?
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        grabSpec6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(score5Pose),
                                new Point(grabPose)
                        )
                )
                .setPathEndTimeoutConstraint(0.99)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        scoreSpec6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(score6Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        grabSample1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(score5Pose),
                                new Point(grabPose)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), grabPose.getHeading())
                .setPathEndTimeoutConstraint(0.99)
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        scoreSample1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(sampleScorePose)
                        )
                )
                .setLinearHeadingInterpolation(grabPose.getHeading(), sampleScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(7)
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(3);
                break;
            case 3:
                // was if retractsubnotbusy
                if (!follower.isBusy()) {
                    follower.followPath(grabSpec1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scoreSpec1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(push, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions and reset the timers of switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        robot.loop();
        follower.update();
        autonomousPathUpdate();
        Pose pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(), pos.getY(), pos.getHeading());
        telemetry.addData("Position", data);
        telemetry.addData("pedro pose", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        robot = new AutonomousRobot(hardwareMap);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        pathTimer = new Timer();
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        buildPaths();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}

