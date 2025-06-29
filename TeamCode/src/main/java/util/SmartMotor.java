package util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class SmartMotor {
    private final DcMotorEx motor;
    private double target;

    public SmartMotor(HardwareMap hardwareMap, String motorName, DcMotorEx.Direction direction, boolean resetEncoder, boolean brakeMode) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (resetEncoder) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (brakeMode) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    public void setTarget(double t) {
        target = t;
    }
    public double getTarget() {
        return target;
    }

    public boolean atTarget(double tolerance) {
        return Math.abs(motor.getCurrentPosition() - target) <= tolerance;
    }

    public boolean atTarget() {
        return motor.getCurrentPosition() == target;
    }
    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getCurrentVelocity() {
        return motor.getVelocity();
    }

    public double getPower() {
        return motor.getPower();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}
