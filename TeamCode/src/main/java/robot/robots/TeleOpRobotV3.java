package robot.robots;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import robot.RobotConstantsAuto;
import robot.subsystems.Deposit;
import robot.subsystems.Extension;
import robot.subsystems.Intake;
import robot.subsystems.Lift;
import util.Drivetrain;
import util.RobotFunction;

import java.util.List;

import static robot.RobotConstantsAuto.INTAKE_ELBOW_DROP_OFF;
import static robot.RobotConstantsAuto.INTAKE_TURRET_DROP_OFF;
import static robot.RobotConstantsAuto.INTAKE_WRIST_DROP_OFF;
import static robot.RobotConstantsTeleOp.*;

public class TeleOpRobotV3 {
    List<LynxModule> allHubs;
    private double currentHeadingTarget = 0;
    private HardwareMap hardwareMap;
    public Lift lift;
    public Extension extension;
    public Deposit deposit;
    public Intake intake;
    // public DcMotorEx fl, bl, fr, br;
    public Gamepad gp1, pgp1, cgp1, gp2, pgp2, cgp2;
    public Drivetrain drivetrain;
    private RobotFunction separateSamples, hangLock, reset;

    public int sampleCycleState = 1;
    public int specCycleState = 2;
    public int sampleModeCycleState = 1;
    public boolean sampleMode = false;
    public boolean highBasket = true;

    public boolean isHanging = false;

    public boolean isFinishedHanging = false;
    private boolean initedDeposit = false;
    private boolean isAutoScoring = false;
    private int timesScored = 0;


    public Timer sample1Timer, sample2Timer, sample3Timer, spec1Timer, spec2Timer, sampleMode1Timer, sampleMode2Timer, sampleMode3Timer, autoScoreTimer;
    public int sampleState1, sampleState2, sampleState3, specState1, specState2, sampleModeState1, sampleModeState2, sampleModeState3, autoScoreState;

    // public Follower follower;
    public double speed = .6;
    public int liftOffset = 0;
    public int grabOffset = 0;
    public double strafeSpeed = .6;
    public double turnSpeed = .2;

    public TeleOpRobotV3(HardwareMap hardwareMap, Gamepad gp1, Gamepad gp2) {
        this.hardwareMap = hardwareMap;
        drivetrain = new Drivetrain(this.hardwareMap);

        this.gp1 = gp1;
        this.pgp1 = new Gamepad();
        this.cgp1 = new Gamepad();

        this.gp2 = gp2;
        this.pgp2 = new Gamepad();
        this.cgp2 = new Gamepad();

        sample1Timer = new Timer();
        sample2Timer = new Timer();
        sample3Timer = new Timer();
        spec1Timer = new Timer();
        spec2Timer = new Timer();
        sampleMode1Timer = new Timer();
        sampleMode2Timer = new Timer();
        sampleMode3Timer = new Timer();
        autoScoreTimer = new Timer();

        separateSamples = new RobotFunction(
                List.of(
                        () -> intake.closeIntakeClaw(),
                        () -> intake.openIntakeClaw()
                ),
                List.of(0.3,0.1)
                //List.of(0.15, 0.1)
        );

        hangLock = new RobotFunction(
                List.of(
                        () -> intake.setTurretPos(INTAKE_TURRET_DROP_OFF),
                        () -> intake.setWristPos(INTAKE_WRIST_HANG_LOCK),
                        () -> intake.setClawIntakePos(INTAKE_CLAW_HANG_LOCK),
                        () -> intake.setElbowIntakePos(INTAKE_ELBOW_HANG_LOCK),
                        () -> intake.setTurretPos(INTAKE_TURRET_HANG_LOCK)
                ),
                List.of(.1,.1,.3,.2,.1)
        );

        reset = new RobotFunction(
                List.of(
                        () -> deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB),
                        () -> lift.setTargetPos(LIFT_SPEC_GRAB)
                ),
                List.of(0.6, 0.1)
        );


        allHubs = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    public void initSubsystems() {
        intake = new Intake(this.hardwareMap);
        deposit = new Deposit(this.hardwareMap);
        // TODO: for real runs we will put this as false
        lift = new Lift(this.hardwareMap, false);
        lift.setTwoMotors(true);
        extension = new Extension(this.hardwareMap, false);
        extension.setDeadZone(true);
        intake.setTurretPos(INTAKE_TURRET_DEFAULT);
        intake.setWristPos(INTAKE_WRIST_DEFAULT);
        intake.setElbowIntakePos(INTAKE_ELBOW_DEFAULT);
        intake.openIntakeClaw();

        deposit.openDepositClaw();
        lift.setTargetPos(LIFT_SPEC_SCORE);
        extension.setTargetPos(EXTENSION_MIN);
    }

    public void start() {
        //follower.startTeleopDrive();
    }

    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        sampleCycle1();
        sampleCycle2();
        sampleCycle3();
        specCycle1();
        specCycle2();
        sampleModeCycle1();
        sampleModeCycle2();
        sampleModeCycle3();
        autoScore();

        extension.loop();
        separateSamples.run();
        hangLock.run();
        lift.loop();
        reset.run();
        //follower.update();
//        follower.setTeleOpMovementVectors(-gp1.left_stick_y*speed,
//                -gp1.left_stick_x*strafeSpeed,
//                -gp1.right_stick_x*turnSpeed);
//        System.out.println(fl.getCurrent(CurrentUnit.AMPS));
//        System.out.println(bl.getCurrent(CurrentUnit.AMPS));
//        System.out.println(br.getCurrent(CurrentUnit.AMPS));
//        System.out.println(fr.getCurrent(CurrentUnit.AMPS));

    }

    public void sampleCycle1() {
        switch (sampleState1) {
            case 1:
                intake.openIntakeClaw();
                intake.setTurretPos(INTAKE_TURRET_DEFAULT);
                extension.setTargetPos(EXTENSION_MAX);
                // intake.setElbowIntakePos(INTAKE_ELBOW_DOWN);
                setSampleState1(2);
                break;
            case 2:
                if (sample1Timer.getElapsedTimeSeconds() > 0.3) {
                    intake.setElbowIntakePos(INTAKE_ELBOW_DOWN);
                    setSampleState1(-1);
                }
                break;
        }
    }
    public void setSampleState1(int x) {
        sampleState1 = x;
        sample1Timer.resetTimer();
    }
    public void startSampleCycle1() {
        setSampleState1(1);
    }

    public void sampleCycle2() {
        switch (sampleState2) {
            case 1:
                intake.closeIntakeClaw();
                setSampleState2(2);
                break;
            case 2:
                if (sample2Timer.getElapsedTimeSeconds() > 0.4) {
                    extension.setTargetPos(EXTENSION_MIN);
                    intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                    setSampleState2(3);
                }
                break;
            case 3:
                if (sample2Timer.getElapsedTimeSeconds() > 1) {
                    intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                    intake.setWristPos(INTAKE_WRIST_DROP_OFF);
                    setSampleState2(-1);
                }
                break;
        }
    }
    public void setSampleState2(int x) {
        sampleState2 = x;
        sample2Timer.resetTimer();
    }
    public void startSampleCycle2() {
        setSampleState2(1);
    }

    public void sampleCycle3() {
        switch (sampleState3) {
            case 1:
                intake.openIntakeClaw();
                setSampleState3(2);
                break;
            case 2:
                if (sample3Timer.getElapsedTimeSeconds() > 0.4) {
                    intake.setTurretPos(.7);
                    // intake.setElbowIntakePos(INTAKE_ELBOW_DEFAULT);
                    intake.setWristPos(INTAKE_WRIST_DEFAULT);
                    setSampleState3(-1);
                }
                break;
        }
    }
    public void setSampleState3(int x) {
        sampleState3 = x;
        sample3Timer.resetTimer();
    }
    public void startSampleCycle3() {
        setSampleState3(1);
    }

    public void specCycle1() {
        switch (specState1) {
            case 1:
                deposit.openDepositClaw();
                intake.setTurretPos(0.7);
                setSpecState1(2);
                break;
            case 2:
                if (spec1Timer.getElapsedTimeSeconds() > 0.3) {
                    deposit.retractLinkage();
                    deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                    setSpecState1(3);
                }
                break;
            case 3:
                if (spec1Timer.getElapsedTimeSeconds() > 0.3) {
                    lift.setTargetPos(LIFT_SPEC_GRAB + grabOffset);
                    setSpecState1(-1);
                }
                break;
        }
    }
    public void setSpecState1(int x) {
        specState1 = x;
        spec1Timer.resetTimer();
    }
    public void startSpecCycle1() {
        setSpecState1(1);
    }

    public void specCycle2() {
        switch (specState2) {
            case 1:
                deposit.setClawDepositPos(DEPOSIT_CLAW_LOOSE);
                intake.setTurretPos(0.7);
                setSpecState2(2);
                break;
            case 2:
                if (spec2Timer.getElapsedTimeSeconds() > 0.1) {
                    lift.setTargetPos(LIFT_SPEC_SCORE+liftOffset);
                    setSpecState2(3);
                }
                break;
            case 3:
                if (spec2Timer.getElapsedTimeSeconds() > 0.3) {
                    deposit.closeDepositClaw();
                    deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_SCORE);
                    setSpecState2(-1);
                }
                break;
        }
    }
    public void setSpecState2(int x) {
        specState2 = x;
        spec2Timer.resetTimer();
    }
    public void startSpecCycle2() {
        setSpecState2(1);
    }

    public void sampleModeCycle1() {
        switch (sampleModeState1) {
            case 1:
                extension.setTargetPos(EXTENSION_MAX);
                intake.setElbowIntakePos(INTAKE_ELBOW_DOWN);
                setSampleModeState1(-1);
                break;
        }
    }
    public void setSampleModeState1(int x) {
        sampleModeState1 = x;
        sampleMode1Timer.resetTimer();
    }
    public void startSampleModeCycle1() {
        setSampleModeState1(1);
    }

    public void sampleModeCycle2() {
        switch (sampleModeState2) {
            case 1:
                intake.closeIntakeClaw();
                // the next are just in case
                deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
                deposit.openDepositClaw();
                lift.setTargetPos(LIFT_TRANSFER);
                setSampleModeState2(2);
                break;
            case 2:
                if (sampleMode2Timer.getElapsedTimeSeconds() > 0.3) {
                    intake.setWristPos(INTAKE_WRIST_DEFAULT);
                    intake.setElbowIntakePos(INTAKE_ELBOW_TRANSFER);
                    intake.setTurretPos(INTAKE_TURRET_TRANSFER);
                    setSampleModeState2(69);
                }

                break;
            case 69:
                if (sampleMode2Timer.getElapsedTimeSeconds() > 0.9) {
                    extension.setTargetPos(EXTENSION_TRANSFER);
                    setSampleModeState2(3);
                }

                break;
            case 3:

                if (sampleMode2Timer.getElapsedTimeSeconds() > 0.4) {
                    deposit.closeDepositClaw();
                    setSampleModeState2(4);
                }

                break;
            case 4:
                if (sampleMode2Timer.getElapsedTimeSeconds() > 0.1) {
                    intake.openIntakeClaw();
                    setSampleModeState2(5);
                }

                break;
            case 5:
                if (sampleMode2Timer.getElapsedTimeSeconds() > 0.3) {
                    deposit.setElbowDepositPos(DEPOSIT_ELBOW_SAMPLE_SCORE); // score
                    if (highBasket) {
                        lift.setTargetPos(LIFT_SAMPLE_HIGH);
                    } else {
                        lift.setTargetPos(LIFT_SAMPLE_LOW);
                    }
                    setSampleModeState2(6);
                }

                break;
            case 6:
                if (sampleMode2Timer.getElapsedTimeSeconds() > 0.5) {
                    intake.setElbowIntakePos(INTAKE_ELBOW_DEFAULT);
                    setSampleModeState2(-1);
                }


        }

    }
    public void setSampleModeState2(int x) {
        sampleModeState2 = x;
        sampleMode2Timer.resetTimer();
    }
    public void startSampleModeCycle2() {
        setSampleModeState2(1);
    }

    public void sampleModeCycle3() {
        switch (sampleModeState3) {
            case 1:
                deposit.openDepositClaw();
                setSampleModeState3(2);
                break;
            case 2:
                if (sampleMode3Timer.getElapsedTimeSeconds() > 0.3) {
                    deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
                    setSampleModeState3(3);
                }
                break;
            case 3:
                if (sampleMode3Timer.getElapsedTimeSeconds() > 0.3) {
                    lift.setTargetPos(LIFT_TRANSFER);
                    setSampleModeState3(-1);
                }
                break;
        }
    }
    public void setSampleModeState3(int x) {
        sampleModeState3 = x;
        sampleMode3Timer.resetTimer();
    }
    public void startSampleModeCycle3() {
        setSampleModeState3(1);
    }

    public void autoScore() {
        // new autoScore: (reset pos when score button is pressed for the first time)
        // drive back, lift goes back down
        // grab new spec, drop off new sample (take time while doing this!)
        // drive back to sub, score, release specimen
        switch (autoScoreState) {
            case 1:
                PathChain backwardPath = new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        // add control points here
                                        new Point(drivetrain.follower.getPose()),
                                        new Point(15, 34, Point.CARTESIAN),
                                        new Point(21, 34, Point.CARTESIAN),
                                        new Point(9,34, Point.CARTESIAN)
                                )
                        )
                        .setPathEndTimeoutConstraint(0)
                        .addParametricCallback(0.1, () -> {
                            deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                            lift.setTargetPos(LIFT_SPEC_GRAB + grabOffset);
                        })
                        .addParametricCallback(0.55, () -> {
                            intake.setWristPos(INTAKE_WRIST_DROP_OFF);
                            intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                            intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                        })
                        .addParametricCallback(0.9, () -> {
                            intake.openIntakeClaw();
                        })
                        .setLinearHeadingInterpolation(drivetrain.follower.getPose().getHeading(), Math.toRadians(0))
                        .build();
                drivetrain.follower.followPath(backwardPath);
                setAutoScoreState(2);
                break;
            case 2:
                if (!drivetrain.follower.isBusy()) {
                    intake.closeIntakeClaw();
                    setAutoScoreState(3);
                }
                break;
            case 3:
                if (autoScoreTimer.getElapsedTimeSeconds() > 0.1) {
                    PathChain forwardPath = new PathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Point(9, 34, Point.CARTESIAN),
                                            new Point(42,79-timesScored, Point.CARTESIAN)
                                    )
                            )
                            .setPathEndTimeoutConstraint(0)
                            .addParametricCallback(0.1, () -> {
                                deposit.setElbowDepositPos(RobotConstantsAuto.DEPOSIT_ELBOW_SPEC_SCORE);
                                lift.setTargetPos(LIFT_SPEC_SCORE + liftOffset);
                            })
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .build();
                    drivetrain.follower.followPath(forwardPath);
                    setAutoScoreState(4);
                }
                break;
            case 4:
                if (!drivetrain.follower.isBusy()) {
                    deposit.openDepositClaw();
                    timesScored++;
                    drivetrain.follower.breakFollowing();
                    isAutoScoring = false;
                    setAutoScoreState(-1);
                }
                break;
        }
    }
    public void setAutoScoreState(int x) {
        autoScoreState = x;
        autoScoreTimer.resetTimer();
    }
    public void startAutoScore() {
        setAutoScoreState(1);
    }
    public void updateControls() {
        pgp1.copy(cgp1);
        cgp1.copy(gp1);
        pgp2.copy(cgp2);
        cgp2.copy(gp2);

        if (isAutoScoring) {
            drivetrain.follower.update();
        } else {
            if (Math.abs(gp1.right_stick_x) > 0 || isHanging || isFinishedHanging) {
                drivetrain.setMovementVectors(gp1.left_stick_y*speed,
                        gp1.left_stick_x*strafeSpeed,
                        gp1.right_stick_x*turnSpeed);
            } else {
                drivetrain.setHeadingLockMovementVectors(gp1.left_stick_y*speed,
                        gp1.left_stick_x*strafeSpeed,
                        gp1.right_stick_x*turnSpeed);
            }
        }

        if (gp1.x) {
            drivetrain.follower.setPose(new Pose(9, 34, Math.toRadians(0)));
        }

        if (gp1.y) {
            isAutoScoring = true;
            startAutoScore();
        }

        if (!sampleMode) {
            if (gp1.b && !pgp1.b) {
                speed = .2;
                strafeSpeed = .2;
                turnSpeed = .15;
                startSampleCycle1();
                sampleCycleState = 2;
                sampleState2 = -1;
                sampleState3 = -1;
            }
        } else {
            // samp
            if (gp1.b && !pgp1.b) {
                speed = .15;
                strafeSpeed = .225;
                turnSpeed = .14;
                startSampleModeCycle1();
                sampleModeCycleState = 2;
                setSampleModeState2(-1);
            }
        }


        if (gp2.dpad_up && !pgp2.dpad_up) {
            liftOffset += 20;
            lift.setTargetPos(LIFT_SPEC_SCORE + liftOffset);
        }

        if (gp2.dpad_down && !pgp2.dpad_down) {
            liftOffset -= 20;
            lift.setTargetPos(LIFT_SPEC_SCORE + liftOffset);
        }

        if (gp2.dpad_left && !pgp2.dpad_left) {
            grabOffset += 10;
            lift.setTargetPos(LIFT_SPEC_GRAB+ grabOffset);
        }

        if (gp2.dpad_right && !pgp2.dpad_right) {
            grabOffset -= 10;
            lift.setTargetPos(LIFT_SPEC_GRAB + grabOffset);
        }

        if (gp2.a && !pgp2.a){
            isHanging = !isHanging;
            lift.setManual(isHanging);
            lift.setTwoMotors(false);
            extension.setHanging(true);

//            extension.setTargetPos(0);
//            intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
            hangLock.start();
        }

        if (gp2.b && !pgp2.b) {
            isFinishedHanging = !isFinishedHanging;
            extension.setIsFinishedHanging(isFinishedHanging);
        }

//        if (gp1.b && !pgp1.b) {
//            highBasket = !highBasket;
//        }

        if (gp1.a && !pgp1.a) {
            sampleMode = !sampleMode;
            if (sampleMode) {
                deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
            } else {
                deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
            }
        }

        if (isHanging) {
            if (isFinishedHanging) {
                lift.setPower(-0.24);
            } else {
                lift.setPower(-gp2.left_stick_y);
            }
        }

        if (gp1.left_trigger>0 && !(pgp1.left_trigger>0)) {
            intake.wristLeft();
        }
        if (gp1.right_trigger>0 && !(pgp1.right_trigger>0)) {
            intake.wristRight();
        }

        if (!sampleMode) {
            if (gp1.right_bumper && !pgp1.right_bumper) {
                if (!initedDeposit) {
                    reset.start();
                    initedDeposit = true;
                }
                if (sampleCycleState == 1) {
                    // extend into sub
                    speed = .2;
                    strafeSpeed = .2;
                    turnSpeed = .15;
                    startSampleCycle1();
                    sampleCycleState = 2;
                } else if (sampleCycleState == 2) {
                    // grab and retract
                    speed = .6;
                    strafeSpeed = 0.6;
                    turnSpeed = 0.2;
                    startSampleCycle2();
                    sampleCycleState = 3;
                } else if (sampleCycleState == 3) {
                    // extend, release sample, retract
                    startSampleCycle3();
                    sampleCycleState = 1;
                    speed = .6;
                    strafeSpeed = 0.6;
                    turnSpeed = 0.2;
                }
            }

            if (gp1.left_bumper && !pgp1.left_bumper) {
                if (specCycleState == 1) {
                    // release, prepare to grab spec
                    speed = .6;
                    strafeSpeed = 0.6;
                    turnSpeed = 0.2;
                    startSpecCycle1();
                    specCycleState = 2;
                } else if (specCycleState == 2) {
                    // grab spec, prepare to score
                    speed = .6;
                    strafeSpeed = 0.6;
                    turnSpeed = 0.2;
                    startSpecCycle2();
                    specCycleState = 1;
                }
            }
        } else {
            if (gp1.right_bumper && !pgp1.right_bumper) {
                if (!initedDeposit) {
                    reset.start();
                    initedDeposit = true;
                }
                if (sampleModeCycleState == 1) {
                    // extend into sub
                    speed = .15;
                    strafeSpeed = .225;
                    turnSpeed = .14;
                    startSampleModeCycle1();
                    sampleModeCycleState = 2;
                } else if (sampleModeCycleState == 2) {
                    // grab and retract
                    speed = 0.7;
                    strafeSpeed = .7;
                    turnSpeed = .25;
                    startSampleModeCycle2();
                    sampleModeCycleState = 3;
                } else if (sampleModeCycleState == 3) {
                    // extend, release sample, retract
                    startSampleModeCycle3();
                    sampleModeCycleState = 1;
                    speed = 0.7;
                    strafeSpeed = .7;
                    turnSpeed = .25;
                }
            }
        }
    }
}
