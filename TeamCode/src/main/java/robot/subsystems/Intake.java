package robot.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static robot.RobotConstants.*;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    public double wristPos = INTAKE_WRIST_DEFAULT;
    private Servo clawIntake;
    private Servo wrist;
    private Servo elbowIntake;

    private double turretPos = INTAKE_TURRET_DEFAULT;

    private Servo turret;
    private HardwareMap hardwareMap;
    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        clawIntake = this.hardwareMap.get(Servo.class, "clawIntake");
        clawIntake.setDirection(Servo.Direction.FORWARD);
        wrist = this.hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        elbowIntake = this.hardwareMap.get(Servo.class, "elbowIntake");
        elbowIntake.setDirection(Servo.Direction.REVERSE);

        turret = this.hardwareMap.get(Servo.class, "turret");
        turret.setDirection(Servo.Direction.FORWARD);
    }
    public double clamp(double x) {
        if (x > 1) {
            return 1;
        } else if (x < -1) {
            return -1;
        } else {
            return x;
        }
    }

    public void loop() {}

    public void closeIntakeClaw() {
        clawIntake.setPosition(INTAKE_CLAW_CLOSED);
    }
    public void openIntakeClaw() {
        clawIntake.setPosition(INTAKE_CLAW_OPEN);
    }
    public void setWristPos(double pos) {
        wristPos = pos;
        wrist.setPosition(wristPos);
    }
    public void setClawIntakePos(double pos){
        clawIntake.setPosition(pos);
    }
    public void setElbowIntakePos(double pos){
        elbowIntake.setPosition(pos);
    }

    //degrees from vert
    public void setWristAngle(double degrees) {
        wristPos = INTAKE_WRIST_DEFAULT + clamp(degrees/90) * -0.5;
        if (wristPos > 1) {
            wristPos = 1;
        } else if (wristPos < 0) {
            wristPos = 0;
        }
        wrist.setPosition(wristPos);
    }

    public void setTurretAngle(double degrees) {
        turretPos = INTAKE_TURRET_DEFAULT + clamp(degrees/90) * -0.37;
        if (turretPos > 1) {
            turretPos = 1;
        } else if (turretPos < 0) {
            turretPos = 0;
        }
        turret.setPosition(turretPos);
    }

    public void setTurretPos(double pos) {
        turretPos = pos;
        turret.setPosition(turretPos);
    }

    public void wristLeft() {
        wristPos += 0.25;
        wrist.setPosition(wristPos);
    }

    public void wristRight() {
        wristPos -= 0.25;
        wrist.setPosition(wristPos);
    }

//    public boolean sampleInClaw() {
//        LynxModule chub = hardwareMap.get(LynxModule.class, "Control Hub");
//        return chub.getCurrent(CurrentUnit.AMPS) > 1;
//    }


}
