package opmode.autonomous;


import static com.pedropathing.util.Constants.setConstants;
import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import robot.subsystems.Light;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import static robot.RobotConstantsAuto.*;

import java.util.ArrayList;

import robot.RobotConstantsTeleOp;
import robot.robots.AutonomousRobot;
import util.PositionCalculator;
import util.VoltageCompFollower;
import vision.SigmaPythonDetector;

@Autonomous(name = "7+0 red real LOBSTER", group="comp")
public class SpecV25 extends OpMode {
    // here, the user can select which one to pickup on. light will be on the entire time.
    // this helps us be more consistent with our positions, making sure we don't score on top of each other
    // 1 is fixed at 66, but the rest go:
    // 2: 78
    // 3: 76
    // 4: 74
    // 5: 72
    // 6: 70
    // 7: 68 (also fixed)

    public AutonomousRobot robot;
    private Light light;
    private VoltageCompFollower follower;
    private int pathState;
    private Timer pathTimer;
    private Timer totalTimer;
    private double totalElapsed = 0;
    private SigmaPythonDetector detector;
    private final Pose startPose = new Pose(9, 66, Math.toRadians(0));
    private PathChain scorePreload, grabSample12, grabSample3, grabSpec1, scoreSpec1, grabSpec2, scoreSpec2, grabSpec3, scoreSpec3, grabSpec4, scoreSpec4, grabSpec5, scoreSpec5, grabSpec6, scoreSpec6;
    private double[] positions1;
    private double[] positions2;
    private double[] positions3;
    private Gamepad gp1, cgp1, pgp1, gp2, cgp2, pgp2;
    private int x1 = 0;
    private int x2 = 1;
    private ArrayList<Integer> intakingPositions = new ArrayList<>();
    private int[] originalPositions = {76, 74, 72, 70, 69};
    private int[] indices = {0, 1, 2, 3, 4};

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(15, 66, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(4.5)
                .setPathEndTimeoutConstraint(0)
                .build();

        grabSample12 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(15.000, 66.000, Point.CARTESIAN),
                                new Point(28.394, 65.713, Point.CARTESIAN),
                                new Point(20.000, 18.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.99)
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_SPEC_GRAB);;
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                    robot.deposit.openDepositClaw();
                })
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(20.000, 18.500, Point.CARTESIAN),
                                new Point(22.000, 16.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-25))
                .setPathEndTValueConstraint(0.995)
                .build();

        grabSpec1 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(22.000, 15.000, Point.CARTESIAN),
                                new Point(23.324, 34.073, Point.CARTESIAN),
                                new Point(18.659, 33.262, Point.CARTESIAN),
                                new Point(9.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setPathEndTValueConstraint(0.93)
                .addParametricCallback(0.8, () -> {
                    robot.intake.openIntakeClaw();
                })
                .setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .build();

        scoreSpec1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 34.000, Point.CARTESIAN),
                                new Point(43.000, intakingPositions.get(0), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        grabSpec2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(42.000, intakingPositions.get(0), Point.CARTESIAN),
                                new Point(20.890, 34, Point.CARTESIAN),
                                new Point(25.758, 34, Point.CARTESIAN),
                                new Point(9.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_SPEC_GRAB);;
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                })
                .addParametricCallback(0.55, () -> {
                    robot.intake.setWristPos(INTAKE_WRIST_DROP_OFF);
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.9, () -> {
                    robot.intake.openIntakeClaw();
                })
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTValueConstraint(0.92)
                .setPathEndTimeoutConstraint(0)
                .build();

        scoreSpec2 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 34.000, Point.CARTESIAN),
                                new Point(43.000, intakingPositions.get(1), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        grabSpec3 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(42.000, intakingPositions.get(1), Point.CARTESIAN),
                                new Point(20.890, 34, Point.CARTESIAN),
                                new Point(25.758, 34, Point.CARTESIAN),
                                new Point(9.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setPathEndTValueConstraint(0.92)
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_SPEC_GRAB);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                })
                .addParametricCallback(0.55, () -> {
                    robot.intake.setWristPos(INTAKE_WRIST_DROP_OFF);
                    robot.intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    robot.intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                })
                .addParametricCallback(0.9, () -> {
                    robot.intake.openIntakeClaw();
                })
                .setZeroPowerAccelerationMultiplier(2)
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(0)
                .build();

        scoreSpec3 = follower.pathBuilder()
                // TODO: the goal here is to push the first hung spec out of the way.
                // That's why the y value is small here. the inertia will push it out.
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(9.000, 34.000, Point.CARTESIAN),
                                new Point(42.000, intakingPositions.get(2), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndTimeoutConstraint(0)
                .build();

        grabSpec4 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(42.000, intakingPositions.get(2), Point.CARTESIAN),
                                new Point(20.890, 34, Point.CARTESIAN),
                                new Point(25.758, 34, Point.CARTESIAN),
                                new Point(9.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setPathEndTValueConstraint(0.92)
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_SPEC_GRAB);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                })
                .setZeroPowerAccelerationMultiplier(2)
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .build();

        scoreSpec4 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(9.000, 34.000, Point.CARTESIAN),
                                new Point(42.000, intakingPositions.get(3), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndTimeoutConstraint(0)
                .build();

        grabSpec5 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(42.000, intakingPositions.get(3), Point.CARTESIAN),
                                new Point(20.890, 34, Point.CARTESIAN),
                                new Point(25.758, 34, Point.CARTESIAN),
                                new Point(9.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setPathEndTValueConstraint(0.92)
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_SPEC_GRAB);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                })
                .setZeroPowerAccelerationMultiplier(2)
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .build();

        scoreSpec5 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(9.000, 34.000, Point.CARTESIAN),
                                new Point(42.000, intakingPositions.get(4), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndTimeoutConstraint(0)
                .build();

        grabSpec6 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(42.000, intakingPositions.get(4), Point.CARTESIAN),
                                new Point(20.890, 34, Point.CARTESIAN),
                                new Point(25.758, 34, Point.CARTESIAN),
                                new Point(9.000, 34.000, Point.CARTESIAN)
                        )
                )
                .addParametricCallback(0.1, () -> {
                    robot.lift.setTargetPos(LIFT_SPEC_GRAB);
                    robot.deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                })

                .setPathEndTValueConstraint(0.92)
                .setZeroPowerAccelerationMultiplier(2)
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .build();

        scoreSpec6 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(9.000, 32.000, Point.CARTESIAN),
                                new Point(42.000, 68.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndTimeoutConstraint(0)
                .build();



    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.7);
                follower.followPath(scorePreload, false);
                // robot.lift.setTargetPos(RobotConstantsTeleOp.LIFT_SPEC_SCORE);
                // robot.deposit.setElbowDepositPos(RobotConstantsTeleOp.DEPOSIT_ELBOW_SPEC_SCORE);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    // robot.prepareGrabSpecimen.start();
                    setPathState(2);
                }
                break;
            case 2:
                if (robot.prepareGrabSpecimen.isFinished()) {
                    follower.setMaxPower(1);
                    FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(2, 0, 0.05, 0);
                    FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(2, 0, 0.05, 0);
                    follower.followPath(grabSample12, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    robot.setPositions(positions1);
                    robot.fastGrab.start();
                    setPathState(4);
                }
                break;
            case 4:
                if (robot.fastGrab.isFinished()) {
                    robot.firstSample.start();
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.firstSample.isFinished()) {
                    robot.setPositions(positions2);
                    robot.fastGrab.start();
                    setPathState(6);
                }
                break;
            case 6:
                if (robot.fastGrab.isFinished()) {
                    robot.fastRetract.start();
                    follower.followPath(grabSample3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (robot.fastRetract.isFinished() && !follower.isBusy()) {
                    robot.setPositions(positions3);
                    robot.fastGrab2.start();
                    setPathState(8);
                }
                break;
            case 8:
                if (robot.fastGrab2.isFinished()) {
                    robot.thirdSample.start();
                    follower.setMaxPower(1);
                    FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(1, 0, 0.01, 0);
                    FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(1, 0, 0.01, 0);
                    follower.followPath(grabSpec1, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(10);
                }
                break;
            case 10:
                if (robot.grabSpecimen.isFinished()) {
                    FollowerConstants.holdPointHeadingScaling = 0.8;
                    FollowerConstants.holdPointTranslationalScaling = 0.8;
                    follower.followPath(scoreSpec1, true);
                    robot.scoreSpecimen.start();
                    light.on();
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && robot.scoreSpecimen.isFinished()) {
                    robot.prepareGrabSpecimen.start();
                    robot.intake.setTurretAngle(90);
                    setPathState(69420);
                }
                break;
            case 69420:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    follower.breakFollowing();
                    setPathState(69);
                }
                break;
            case 69:
                double[] distances1 = detector.getDistances();
                if (distances1[0] != 0 && distances1[1] != 0) {
                    double[] positions = detector.getPositions();
                    robot.setPositions(positions);
                    // robot.prepareGrabSpecimen.start();
                    robot.fastGrab.start();
                    setPathState(12);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    // robot.prepareGrabSpecimen.start();
                    setPathState(12);
                }
                break;
            case 12:
                if (robot.fastGrab.isFinished() && robot.prepareGrabSpecimen.isFinished()) {
                    robot.retractExtension.start();
                    light.off();
                    follower.setMaxPower(1);
                    FollowerConstants.holdPointHeadingScaling = 0.35;
                    FollowerConstants.holdPointTranslationalScaling = 0.35;
                    follower.followPath(grabSpec2, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (robot.retractExtension.isFinished() && !follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(14);
                }
                break;
            case 14:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    FollowerConstants.holdPointHeadingScaling = 0.8;
                    FollowerConstants.holdPointTranslationalScaling = 0.8;
                    // TODO: false and true both seem to work
                    follower.followPath(scoreSpec2, true);
                    light.on();
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy() && robot.scoreSpecimen.isFinished()) {
                    robot.prepareGrabSpecimen.start();
                    robot.intake.setTurretAngle(90);
                    setPathState(42069);
                }
                break;
            case 42069:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    follower.breakFollowing();
                    setPathState(440);
                }
                break;
            case 440:
                double[] distances2 = detector.getDistances();
                if (distances2[0] != 0 && distances2[1] != 0) {
                    double[] positions = detector.getPositions();
                    robot.setPositions(positions);
                    robot.fastGrab.start();
                    setPathState(16);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    setPathState(16);
                }
                break;
            case 16:
                if (robot.fastGrab.isFinished() && robot.prepareGrabSpecimen.isFinished()) {
                    robot.retractExtension.start();
                    detector.off();
                    light.off();
                    follower.setMaxPower(1);
                    FollowerConstants.holdPointHeadingScaling = 0.35;
                    FollowerConstants.holdPointTranslationalScaling = 0.35;
                    follower.followPath(grabSpec3, true);
                    setPathState(17);
                }
                break;
            case 17:
                if (robot.retractExtension.isFinished() && !follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(18);
                }
                break;
            case 18:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec3, false);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(20);
                }
                break;
            case 20:
                if (robot.prepareGrabSpecimen.isFinished()) {
                    follower.followPath(grabSpec4, true);
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(22);
                }
                break;
            case 22:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec4, false);
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(24);
                }
                break;
            case 24:
                if (robot.prepareGrabSpecimen.isFinished()) {
                    follower.followPath(grabSpec5, true);
                    setPathState(25);
                }
                break;
            case 25:
                if (!follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(26);
                }
                break;
            case 26:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec5, false);
                    setPathState(27);
                }
                break;
            case 27:
                if (!follower.isBusy()) {
                    robot.prepareGrabSpecimen.start();
                    setPathState(28);
                }
                break;
            case 28:
                if (robot.prepareGrabSpecimen.isFinished()) {
                    follower.followPath(grabSpec6, true);
                    setPathState(29);
                }
                break;
            case 29:
                if (!follower.isBusy()) {
                    robot.grabSpecimen.start();
                    setPathState(30);
                }
                break;
            case 30:
                if (robot.grabSpecimen.isFinished()) {
                    robot.scoreSpecimen.start();
                    follower.followPath(scoreSpec6, false);
                    setPathState(31);
                }
                break;
            case 31:
                if (!follower.isBusy()) {
                    totalElapsed = totalTimer.getElapsedTimeSeconds();
                    robot.deposit.openDepositClaw();
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

        telemetry.addData("total elapsed time", totalTimer.getElapsedTimeSeconds());
        telemetry.addData("final elapsed time", totalElapsed);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        totalTimer = new Timer();
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        positions1 = PositionCalculator.getPositions(-5, 21, 0);
        positions2 = PositionCalculator.getPositions(5, 20, 0);
        positions3 = PositionCalculator.getPositions(0, 22, -25);
        detector = new SigmaPythonDetector(hardwareMap, "red sample");
        gp1 = gamepad1;
        pgp1 = new Gamepad();
        cgp1 = new Gamepad();
        gp2 = gamepad2;
        pgp2 = new Gamepad();
        cgp2 = new Gamepad();
        light = new Light(hardwareMap);
        light.off();

        try {
            sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot = new AutonomousRobot(hardwareMap);
        robot.lift.setTwoMotors(true);
    }

    @Override
    public void init_loop() {
        pgp1.copy(cgp1);
        cgp1.copy(gp1);
        pgp2.copy(cgp2);
        cgp2.copy(gp2);

        if (gp1.dpad_left && !pgp1.dpad_left) {
            x1--;
        }
        if (gp1.dpad_right && !pgp1.dpad_right) {
            x1++;
        }
        if (gp2.dpad_left && !pgp2.dpad_left) {
            x2--;
        }
        if (gp2.dpad_right && !pgp2.dpad_right) {
            x2++;
        }

        // ArrayList<Integer> intakingPositions2 = new ArrayList<>();

        intakingPositions.clear();
        intakingPositions.add(originalPositions[x1]);
        intakingPositions.add(originalPositions[x2]);
        for (int i: indices) {
            if (i != x1 && i != x2) {
                intakingPositions.add(originalPositions[i]);
            }
        }

        // intakingPositions = intakingPositions2;

        robot.deposit.setElbowDepositPos(0.85);
        robot.lift.setTargetPos(300);
        robot.loop();
        telemetry.addLine("Original Values: 78, 76, 74, 72, 70");
        telemetry.addLine("Choose two indices to intake from without repeats:");
        telemetry.addData("x1", x1);
        telemetry.addData("x2", x2);
        telemetry.update();
    }

    @Override
    public void start() {
        totalTimer.resetTimer();
        detector.on();
        buildPaths();
        setPathState(0);
    }
}
