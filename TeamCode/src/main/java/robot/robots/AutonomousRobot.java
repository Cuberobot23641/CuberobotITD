package robot.robots;


import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import robot.subsystems.Deposit;
import robot.subsystems.Extension;
import robot.subsystems.Intake;
import robot.subsystems.Lift;
import util.RobotFunction;

import java.util.List;
import static robot.RobotConstants.*;

public class AutonomousRobot {
    List<LynxModule> allHubs;
    private HardwareMap hardwareMap;
    public Lift lift;
    public Extension extension;
    public Deposit deposit;
    public Intake intake;
    public RobotFunction grabSample, retractExtension, extendExtension, grabSpecimen, scoreSpecimen, prepareGrabSpecimen, transfer, fastRetract, clickbaitGrab, fastGrab, transferUp, transferDown, transferIn;

    public double extensionInches;
    public double turretAngle;
    public double wristAngle;

    public AutonomousRobot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        intake = new Intake(this.hardwareMap);
        deposit = new Deposit(this.hardwareMap);
        lift = new Lift(this.hardwareMap, true);
        extension = new Extension(this.hardwareMap, true);

        // init positions
        intake.setTurretAngle(INTAKE_TURRET_DROP_OFF);
        intake.setWristPos(INTAKE_WRIST_DROP_OFF);
        intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
        intake.openIntakeClaw();

        deposit.closeDepositClaw();
        deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_SCORE);
        deposit.retractLinkage();

        //lift.setTargetPos(LIFT_TRANSFER);
        // extension.setTargetPos(EXTENSION_MIN);

        grabSample = new RobotFunction(
                List.of(
                        () -> intake.setElbowIntakePos(INTAKE_ELBOW_DOWN),
                        () -> intake.closeIntakeClaw()
                ),
                List.of(0.2, 0.3)
        );


        retractExtension = new RobotFunction(
                List.of(
                        () -> {
                            intake.setElbowIntakePos(INTAKE_ELBOW_IN);
                            intake.setWristPos(INTAKE_WRIST_DEFAULT);
                            intake.setTurretPos(INTAKE_TURRET_DEFAULT);
                            extension.setTargetPos(EXTENSION_MIN);
                        },
                        () -> {
                            intake.setWristPos(INTAKE_WRIST_DROP_OFF);
                            intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                            intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                        }
                ),
                List.of(1.0, 0.1)
        );

        fastRetract = new RobotFunction(
                List.of(
                        () -> {
                            intake.setWristPos(INTAKE_WRIST_DROP_OFF);
                            intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                            intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                        },
                        () -> extension.setTargetPos(EXTENSION_MIN),
                        () -> intake.openIntakeClaw()
                ),
                List.of(0.15, 0.15, 0.2)
        );

        extendExtension = new RobotFunction(
                List.of(
                        () -> {
                            intake.setWristAngle(wristAngle);
                            intake.setTurretAngle(turretAngle);
                            intake.setElbowIntakePos(INTAKE_ELBOW_HOVER);
                            extension.setTargetInches(extensionInches);
                        }
                ),
                List.of(0.5)
        );

        grabSpecimen = new RobotFunction(
                List.of(
                        () -> {
                            deposit.closeDepositClaw();
                            intake.openIntakeClaw();
                            // added this here hopefully it helps
                            intake.setTurretPos(0);
                        }
                ),
                List.of(0.1)
        );

        scoreSpecimen = new RobotFunction(
                List.of(
                        () -> lift.setTargetPos(LIFT_SPEC_SCORE),
                        () -> deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_SCORE),
                        () -> deposit.extendLinkage()
                ),
                List.of(0.3, 0.2, 0.1)
        );

        prepareGrabSpecimen = new RobotFunction(
                List.of(
                        () -> deposit.openDepositClaw(),
                        () -> {
                            deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
                            deposit.retractLinkage();
                        },
                        () -> lift.setTargetPos(LIFT_SPEC_GRAB)
                ),
                List.of(0.2, 0.3, 0.1)
        );

        // this assumes that you have already grabbed
        transfer = new RobotFunction(
                List.of(
                        () -> {
                            deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
                            intake.setElbowIntakePos(INTAKE_ELBOW_TRANSFER);
                            intake.setTurretPos(INTAKE_TURRET_TRANSFER);
                            intake.setWristPos(INTAKE_WRIST_DEFAULT);
                        },
                        () -> extension.setTargetPos(EXTENSION_TRANSFER),
                        () -> deposit.closeDepositClaw(),
                        () -> intake.openIntakeClaw(),
                        () -> deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB),
                        () -> {
                            deposit.openDepositClaw();
                            extension.setTargetInches(extensionInches);
                            intake.setWristAngle(wristAngle);
                            intake.setTurretAngle(turretAngle);
                            intake.setElbowIntakePos(INTAKE_ELBOW_HOVER);
                        }
                ),
                List.of(0.5, 0.4, 0.1, 0.1, 0.5, 0.5)
        );

        clickbaitGrab = new RobotFunction(
                List.of(
                        () -> {
                            intake.setWristAngle(wristAngle);
                            intake.setTurretAngle(turretAngle);
                            intake.setElbowIntakePos(INTAKE_ELBOW_HOVER);
                            extension.setTargetInches(extensionInches);
                        },
                        () -> intake.setElbowIntakePos(INTAKE_ELBOW_DOWN),
                        () -> intake.closeIntakeClaw()
                ),
                List.of(0.5, 0.2, 0.3)
        );

        fastGrab = new RobotFunction(
                List.of(
                        () -> {
                            intake.setWristAngle(wristAngle);
                            intake.setTurretAngle(turretAngle);
                            intake.setElbowIntakePos(INTAKE_ELBOW_HOVER);
                            extension.setTargetInches(extensionInches);
                        },
                        () -> intake.setElbowIntakePos(INTAKE_ELBOW_DOWN),
                        () -> intake.closeIntakeClaw()
                ),
                List.of(0.5, 0.2, 0.3)
        );

        transferUp = new RobotFunction(
                List.of(
                        () -> lift.setTargetPos(LIFT_SAMPLE_HIGH),
                        () -> deposit.setElbowDepositPos(DEPOSIT_ELBOW_SAMPLE_SCORE)
                ),
                List.of(0.8, 0.4)
        );

        transferDown = new RobotFunction(
                List.of(
                        () -> deposit.openDepositClaw(),
                        () -> deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER),
                        () -> lift.setTargetPos(LIFT_SAMPLE_HIGH)
                ),
                List.of(0.2, 0.4, 0.6)
        );

        transferIn = new RobotFunction(
                List.of(
                        () -> {
                            deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
                            intake.setElbowIntakePos(INTAKE_ELBOW_TRANSFER);
                            intake.setTurretPos(INTAKE_TURRET_TRANSFER);
                            intake.setWristPos(INTAKE_WRIST_DEFAULT);
                        },
                        () -> extension.setTargetPos(EXTENSION_TRANSFER),
                        () -> deposit.closeDepositClaw(),
                        () -> intake.openIntakeClaw()

                ),
                List.of(0.5, 0.4, 0.1, 0.1)
        );


//        allHubs = this.hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
    }

    public void loop() {
//        for (LynxModule hub : allHubs) {
//            hub.clearBulkCache();
//        }

        grabSample.run();
        retractExtension.run();
        extendExtension.run();
        grabSpecimen.run();
        scoreSpecimen.run();
        prepareGrabSpecimen.run();
        transfer.run();
        clickbaitGrab.run();
        fastRetract.run();
        fastGrab.run();
        transferIn.run();

        extension.loop();
        lift.loop();
    }

    public void setTurretAngle(double x) {
        turretAngle = x;
    }

    public void setWristAngle(double x) {
        wristAngle = x;
    }

    public void setExtensionInches(double x) {
        extensionInches = x;
    }

    public void setPositions(double[] positions) {
        extensionInches = positions[0];
        turretAngle = positions[1];
        wristAngle = positions[2];
    }
}
