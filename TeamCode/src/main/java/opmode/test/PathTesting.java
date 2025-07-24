package opmode.test;

import static com.pedropathing.util.Constants.setConstants;
import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import static robot.RobotConstantsAuto.*;

import robot.RobotConstantsTeleOp;
import robot.robots.AutonomousRobot;
import util.PositionCalculator;
import util.fsm.State;
import util.fsm.StateMachine;
import util.fsm.Transition;
import util.pathgenerator.ControlPoint;
import util.pathgenerator.PathGenerator;
import util.pathgenerator.SafePathGenerator;
import util.pathgenerator.TargetPose;
import vision.SigmaPythonDetector;

@Autonomous(name = "testing fsm and other stuff")
public class PathTesting extends OpMode {
    private Follower follower;
    private PathGenerator pathGenerator;
    private SafePathGenerator safePathGenerator;
    private StateMachine stateMachine;
    private int i = 0;
    private final Pose startPose = new Pose(9, 66, Math.toRadians(0));

    public void followNextPath() {
        follower.followPath(pathGenerator.getPaths().get(i));
        i++;
    }

    @Override
    public void loop() {
        follower.update();
        stateMachine.run();
    }

    @Override
    public void init() {
        // follower stuff
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        // path stuff
        pathGenerator = new PathGenerator(
                new TargetPose(9.0, 66.0, Math.toRadians(0.0)),
                new TargetPose(40.0, 66.0, Math.toRadians(0.0)),
                new ControlPoint(28.0, 66.0),
                new TargetPose(20.0, 18.0, Math.toRadians(0.0))
        );
        pathGenerator.generatePaths();

        safePathGenerator = new SafePathGenerator(
                new TargetPose(9.0, 66.0, Math.toRadians(0.0)),
                new TargetPose(40.0, 66.0, Math.toRadians(0.0)),
                new TargetPose(20.0, 18.0, Math.toRadians(0.0))
        );
        safePathGenerator.generatePaths();
        // state stuff
        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> followNextPath())
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> followNextPath())
                        .transition(new Transition(() -> !follower.isBusy()))
        );
    }

    @Override
    public void init_loop() {
        // follower.update();
        // robot.update();
    }

    @Override
    public void start() {
        stateMachine.start();
    }
}
