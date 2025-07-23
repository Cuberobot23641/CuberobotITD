package robot.robots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import robot.RobotConstantsTeleOp;
import robot.subsystems.Deposit;
import robot.subsystems.Extension;
import robot.subsystems.Intake;
import robot.subsystems.Lift;
import util.RobotFunction;

import java.util.List;
import static robot.RobotConstantsAuto.*;

public class AutonomousRobot {
    List<LynxModule> allHubs;
    private HardwareMap hardwareMap;
    public Lift lift;
    public Extension extension;
    public Deposit deposit;
    public Intake intake;
    public RobotFunction fastGrab2, grabSample, retractExtension, extendExtension, grabSpecimen, scoreSpecimen, prepareGrabSpecimen, transfer, fastRetract, clickbaitGrab, fastGrab, transferUp, transferDown, transferIn, thirdSample, firstSample, subGrab, retractSub, newScoreSpecimen;

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
        intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
        intake.setWristPos(INTAKE_WRIST_DROP_OFF);
        intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
        intake.openIntakeClaw();

        deposit.closeDepositClaw();
        deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_SCORE);
        deposit.retractLinkage();

        lift.setTargetPos(LIFT_TRANSFER);
        extension.setTargetPos(EXTENSION_MIN);

        grabSample = new RobotFunction(
                List.of(
                        () -> intake.setElbowIntakePos(INTAKE_ELBOW_DOWN),
                        () -> intake.closeIntakeClaw()
                ),
                List.of(0.2, 0.3)
        );

        thirdSample = new RobotFunction(
                List.of(
                        () -> extension.setTargetPos(EXTENSION_MIN),
                        () -> {
                            intake.setWristPos(INTAKE_WRIST_DROP_OFF);
                            intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                            intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                        }
                ),
                List.of(0.15, 0.1)
        );


        retractExtension = new RobotFunction(
                List.of(
                        () -> {
                            intake.setElbowIntakePos(INTAKE_ELBOW_IN);
                            intake.setWristPos(INTAKE_WRIST_DEFAULT);
                            // changed this from default
                            intake.setTurretPos(.83);
                            extension.setTargetPos(EXTENSION_MIN);
                        }
                        // TODO: was here and was 0.1s
//                        () -> {
//                            intake.setWristPos(INTAKE_WRIST_DROP_OFF);
//                            intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
//                            intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
//                        }
                ),
                List.of(1.0)
        );

        retractSub = new RobotFunction(
                List.of(
                        () -> {
                            intake.setElbowIntakePos(INTAKE_ELBOW_IN+0.04);
                            intake.setWristPos(INTAKE_WRIST_DEFAULT);
                            // changed this from default
                            intake.setTurretPos(.83);
                            extension.setTargetPos(EXTENSION_MIN);
                        }
                        // TODO: was here and was 0.1s
//                        () -> {
//                            intake.setWristPos(INTAKE_WRIST_DROP_OFF);
//                            intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
//                            intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
//                        }
                ),
                List.of(1.0)
        );

        fastRetract = new RobotFunction(
                List.of(
                        () -> {
                            extension.setTargetPos(EXTENSION_MIN);
                            intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                            intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                            intake.setWristPos(INTAKE_WRIST_DROP_OFF);
                        },

//                        () -> {
//                            intake.setWristPos(INTAKE_WRIST_DROP_OFF);
//                            intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
//                        },
//                        () -> extension.setTargetPos(EXTENSION_MIN),
                        () -> intake.openIntakeClaw()
                ),
                List.of(0.165, 0.15)
        );

        firstSample = new RobotFunction(
                List.of(
                        () -> {
                            extension.setTargetPos(EXTENSION_MIN);
                            intake.setWristPos(INTAKE_WRIST_DROP_OFF);
                            intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                            intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                        },
                        () -> {
                            intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                        },
//                        () -> extension.setTargetPos(EXTENSION_MIN),
                        () -> intake.openIntakeClaw()
                ),
                List.of(0.13, 0.13, 0.1)
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
                            deposit.setClawDepositPos(DEPOSIT_CLAW_LOOSE);
                            // added this here hopefully it helps
                            intake.setTurretPos(0);
                        }
                        // () -> intake.openIntakeClaw()
                ),
                // was all together and set to 0.1
                // TODO: was 0.05 and 0.08
                List.of(0.05)
        );

        scoreSpecimen = new RobotFunction(
                List.of(
                        () -> lift.setTargetPos(LIFT_SPEC_SCORE),
                        () -> {deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_SCORE);
                            deposit.closeDepositClaw();},
                        () -> deposit.extendLinkage()
                ),
                List.of(0.3, 0.2, 0.1)
        );

        newScoreSpecimen = new RobotFunction(
                List.of(
                        () -> lift.setTargetPos(RobotConstantsTeleOp.LIFT_SPEC_SCORE),
                        () -> {deposit.setElbowDepositPos(RobotConstantsTeleOp.DEPOSIT_ELBOW_SPEC_SCORE);
                            deposit.closeDepositClaw();}
                ),
                List.of(0.3, 0.2)
        );

        prepareGrabSpecimen = new RobotFunction(
                // TODO: MAKE THIS SMALLER
                List.of(
                        () ->{ deposit.openDepositClaw();
                            deposit.setElbowDepositPos(0.73);
                        }
//                        () -> {
//                            deposit.setElbowDepositPos(DEPOSIT_ELBOW_SPEC_GRAB);
//                            deposit.retractLinkage();
//                        },
//                        () -> lift.setTargetPos(LIFT_SPEC_GRAB)
                ),
                // was 0.2, making small optimization
                //0.3, 0.1
                List.of(0.01)
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
                            //intake.setElbowIntakePos(INTAKE_ELBOW_HOVER);
                            extension.setTargetInches(extensionInches);
                        },
                        () -> intake.setElbowIntakePos(INTAKE_ELBOW_HOVER),
                        () -> intake.setElbowIntakePos(INTAKE_ELBOW_DOWN),
                        () -> intake.closeIntakeClaw()
                ),
                List.of(0.15, 0.35, 0.15, 0.25)
        );

        fastGrab2 = new RobotFunction(
                List.of(
                        () -> {
                            intake.setWristAngle(wristAngle);
                            intake.setTurretAngle(turretAngle);
                            intake.setElbowIntakePos(0.3);
                            extension.setTargetInches(extensionInches);
                        },
                        () -> intake.setElbowIntakePos(INTAKE_ELBOW_DOWN),
                        () -> intake.closeIntakeClaw()
                ),
                List.of(0.5, 0.15, 0.27)
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

        subGrab = new RobotFunction(
                List.of(
                        () -> {
                            intake.setWristAngle(wristAngle);
                            intake.setTurretAngle(turretAngle);
                            //intake.setElbowIntakePos(INTAKE_ELBOW_HOVER);
                            extension.setTargetInches(extensionInches);
                        },
                        () -> intake.setElbowIntakePos(INTAKE_ELBOW_HOVER),
                        () -> intake.setElbowIntakePos(INTAKE_ELBOW_DOWN),
                        () -> intake.setClawIntakePos(0.63)
                ),
                List.of(0.15, 0.5, 0.25, 0.34)
        );


        allHubs = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

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
        thirdSample.run();
        firstSample.run();
        fastGrab2.run();
        subGrab.run();
        retractSub.run();
        newScoreSpecimen.run();

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
