package robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import util.PIDFController;
import static robot.RobotConstantsAuto.*;

public class Extension {
    public DcMotorEx extension = null;

    public HardwareMap hardwareMap;

    public boolean hanging = false;


    public static final int minPos = 0;
    private boolean deadZoneOn;

    public static final int midPos = 350;

    public static final int maxPos = 700;
    public boolean resetEncoder = true;

    public static double target = 0;
    public static double kS = 0.0001;
    public static int errorThreshold = 10;

    public PIDFController controller;
    public Extension(HardwareMap hardwareMap, boolean resetEncoder) {
        controller = new PIDFController(0.008, 0, 0.0004, 0.00);
        this.hardwareMap = hardwareMap;
        this.resetEncoder = resetEncoder;
        extension = this.hardwareMap.get(DcMotorEx.class, "extension");
        extension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDeadZone(boolean x) {
        deadZoneOn = x;
    }



    public void loop() {
        if (hanging) {
            extension.setPower(-0.3);
        } else if (deadZoneOn && target == 0 && Math.abs(extension.getCurrentPosition()) < 30) {
            extension.setPower(-0.06);
        } else {
            int currentPos = extension.getCurrentPosition();
            double pwr = controller.calculate(currentPos, target);
            pwr += kS * Math.signum(target - currentPos);
            if (Math.abs(target-currentPos) > errorThreshold) {
                pwr += kS * Math.signum(target - currentPos);
            }
            extension.setPower(pwr);
        }

        System.out.println(extension.getCurrentPosition());
    }

    public void setHanging(boolean x) {
        hanging = x;
    }


    public void setTargetPos(double targetPos){
        target = targetPos;
    }

    public int getExtensionPos() {
        return extension.getCurrentPosition();
    }

    public void minPos() {
        setTargetPos(EXTENSION_MIN);
    }

    public void midPos() {
        setTargetPos(EXTENSION_MID);
    }

    public void maxPos() {
        setTargetPos(EXTENSION_MAX);
    }

    public void setTargetInches(double inches) {
        setTargetPos(32.13 * inches + 3.45);
    }
}

