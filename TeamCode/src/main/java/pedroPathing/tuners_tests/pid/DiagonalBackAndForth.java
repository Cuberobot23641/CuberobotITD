package pedroPathing.tuners_tests.pid;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import robot.robots.AutonomousRobot;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "diagonal Back And Forth", group = "PIDF Tuning")
public class DiagonalBackAndForth extends OpMode {
    GoBildaPinpointDriver odo;
    private int timesUnready = 0;
    private AutonomousRobot robot;
    private Telemetry telemetryA;

    public static double DISTANCE = 30;

    private boolean forward = true;

    private Follower follower;

    private Path forwards;
    private Path backwards;

    /**1
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        // odo.recalibrateIMU();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,DISTANCE, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(DISTANCE,DISTANCE, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);

        follower.followPath(forwards);

        try {
            sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot = new AutonomousRobot(hardwareMap);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go side to side continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        robot.loop();
        follower.update();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }

    @Override
    public void init_loop() {
        if (odo.getDeviceStatus().equals(GoBildaPinpointDriver.DeviceStatus.NOT_READY)) {
            timesUnready++;
        }
        telemetry.addData("status", odo.getDeviceStatus());
        telemetry.addData("times unready", timesUnready);
        telemetry.update();
    }
}
