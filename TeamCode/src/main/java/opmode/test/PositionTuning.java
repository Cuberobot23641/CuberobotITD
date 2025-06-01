package opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import robot.robots.AutonomousRobot;
import robot.subsystems.Deposit;
import robot.subsystems.Extension;
import robot.subsystems.Intake;
import robot.subsystems.Lift;
import static robot.RobotConstants.*;

@Config
@TeleOp
public class PositionTuning extends OpMode {

    public AutonomousRobot robot;
    public static double clawIntakePos = INTAKE_CLAW_OPEN;
    public static double clawDepositPos = DEPOSIT_CLAW_OPEN;
    public static double wristPos = INTAKE_WRIST_DEFAULT;
    public static double turretPos = INTAKE_TURRET_DEFAULT;
    public static double linkagePos = DEPOSIT_LINKAGE_RETRACT;
    public static double elbowIntakePos = INTAKE_ELBOW_DEFAULT;
    public static double elbowDepositPos = DEPOSIT_ELBOW_SPEC_SCORE;

    public static double liftTargetPos = 0;
    public static double extensionTargetPos = 0;

    public static boolean intakeOn = false;
    public static boolean depositOn = false;
    public static boolean extensionOn = false;
    public static boolean liftOn = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new AutonomousRobot(hardwareMap);
    }

    @Override
    public void loop() {

        robot.loop();

        if (intakeOn) {
            robot.intake.setClawIntakePos(clawIntakePos);
            robot.intake.setElbowIntakePos(elbowIntakePos);
            robot.intake.setWristPos(wristPos);
            robot.intake.setTurretPos(turretPos);
        }

        if (depositOn) {
            robot.deposit.setElbowDepositPos(elbowDepositPos);
            robot.deposit.setClawDepositPos(clawDepositPos);
            robot.deposit.setLinkagePos(linkagePos);
        }

        if (liftOn) {
            robot.lift.setTargetPos(liftTargetPos);
        }

        if (extensionOn) {
            robot.extension.setTargetPos(extensionTargetPos);
        }

        telemetry.update();
    }
}
