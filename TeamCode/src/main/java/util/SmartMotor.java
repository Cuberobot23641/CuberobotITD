package util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SmartMotor {
    private final DcMotorEx motor;

    public SmartMotor(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction, boolean resetEncoder, boolean brakeMode) {
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

    public void setPower(double power) { motor.setPower(power); }
    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getCurrentVelocity() {
        return motor.getVelocity();
    }

    public double getPower() {
        return motor.getPower();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }
}
