package util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private DcMotorEx fl, bl, fr, br;
    private HardwareMap hardwareMap;
    public Drivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        fl = this.hardwareMap.get(DcMotorEx.class, "front_left_drive");
        bl = this.hardwareMap.get(DcMotorEx.class, "back_left_drive");
        fr = this.hardwareMap.get(DcMotorEx.class, "front_right_drive");
        br = this.hardwareMap.get(DcMotorEx.class, "back_right_drive");
//        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
//        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
//        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
//        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setMovementVectors(double forward, double strafe, double heading) {
        double y = -forward; // Remember, Y stick value is reversed
        double x = strafe * 1.1; // Counteract imperfect strafing
        double rx = heading;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        // maybe only if we are strafing right?
//        double frontLeftPower = (y + 1.4*x + rx) / denominator;
        double frontLeftPower = (y + 1.3*x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
//        double backLeftPower = (y - 1.4*x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
    }
}

