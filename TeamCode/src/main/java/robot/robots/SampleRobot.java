package robot.robots;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import robot.subsystems.Deposit;
import robot.subsystems.Extension;
import robot.subsystems.Intake;
import robot.subsystems.Lift;
import util.RobotFunction;

import java.util.List;
import static robot.RobotConstantsAuto.*;

public class SampleRobot {
    List<LynxModule> allHubs;
    private HardwareMap hardwareMap;
    public Lift lift;
    public Extension extension;
    public Deposit deposit;
    public Intake intake;
    public RobotFunction grabSample, extendExtension, retractExtension, transfer, extendLift, retractLift, scoreSample, fastGrab, transfer2;

    public double extensionInches;
    public double turretAngle;
    public double wristAngle;

    public SampleRobot(HardwareMap hardwareMap) {
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

        lift.setTargetPos(LIFT_TRANSFER);
        extension.setTargetPos(EXTENSION_MIN);

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
                            extension.setTargetPos(EXTENSION_MIN);
                            intake.setWristPos(INTAKE_WRIST_DROP_OFF);
                            intake.setTurretPos(INTAKE_TURRET_DROP_OFF);
                            intake.setElbowIntakePos(INTAKE_ELBOW_DROP_OFF);
                        }
                ),
                List.of(0.5)
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

        // this assumes that you have already grabbed
        transfer = new RobotFunction(
                List.of(
                        () -> {
                            deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
                            intake.setElbowIntakePos(INTAKE_ELBOW_TRANSFER);
                            intake.setTurretPos(INTAKE_TURRET_TRANSFER);
                            intake.setWristPos(INTAKE_WRIST_DEFAULT);
                            extension.setTargetPos(600);
                        },
                        () -> extension.setTargetPos(EXTENSION_TRANSFER),
                        () -> deposit.closeDepositClaw(),
                        () -> intake.openIntakeClaw()
                ),
                // was 0.5, 0.4
                List.of(0.4, 0.4, 0.1, 0.1)
        );

        transfer2 = new RobotFunction(
                List.of(
                        () -> {
                            deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER);
                            intake.setElbowIntakePos(INTAKE_ELBOW_TRANSFER);
                            intake.setTurretPos(INTAKE_TURRET_TRANSFER);
                            intake.setWristPos(INTAKE_WRIST_DEFAULT);
                            extension.setTargetPos(600);
                        },
                        () -> extension.setTargetPos(EXTENSION_TRANSFER),
                        () -> deposit.closeDepositClaw(),
                        () -> intake.openIntakeClaw()
                ),
                // was 0.5, 0.4
                List.of(0.5, 0.2, 0.1, 0.1)
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
                List.of(0.5, 0.15, 0.25)
        );

        extendLift = new RobotFunction(
                List.of(
                        () -> lift.setTargetPos(LIFT_SAMPLE_HIGH),
                        () -> deposit.setElbowDepositPos(DEPOSIT_ELBOW_SAMPLE_SCORE)
                ),
                List.of(0.85, 0.45)
        );

        retractLift = new RobotFunction(
                List.of(
                        () -> lift.setTargetPos(LIFT_TRANSFER),
                        () -> deposit.setElbowDepositPos(DEPOSIT_ELBOW_TRANSFER)
                ),
                List.of(0.3, 0.5)
        );

        scoreSample = new RobotFunction(
                List.of(
                        () -> deposit.openDepositClaw()
                ),
                List.of(0.15)
        );
    }

    public void loop() {

        grabSample.run();
        retractExtension.run();
        extendExtension.run();
        scoreSample.run();
        transfer.run();
        fastGrab.run();
        extendLift.run();
        retractLift.run();
        transfer2.run();

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
