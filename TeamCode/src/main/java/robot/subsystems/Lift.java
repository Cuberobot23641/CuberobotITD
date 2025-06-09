package robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import util.PIDFController;

public class Lift {
    public DcMotorEx liftLeft = null;
    public DcMotorEx liftRight = null;
    public DcMotorEx liftExtra = null;

    public HardwareMap hardwareMap;

    public boolean manual = false;
    public double power = 0;

    // TODO: too lazy to put these in lol


    // vspec was 695
    public static final int verticalSpecimen = 770;//2210;//1900;//625;//1450;

    // was 1050
    public static final int verticalSpecimenUp = 1160;//2210;//1900;//625;//1450;

    // was 725
    public static final int verticalSpecimenAuto = 820;//625;
    public static final int verticalSpecimenUpAuto = 1220 ;//625;

    public static final int verticalForwardSpecimen = 1150;
    public static final int verticalForwardSpecimenDown = 820;
    public static final int verticalBucket = 2300;//2820;
    public boolean resetEncoder = true;

    public static double target = 0;

    public PIDFController controller;
    public int errorThreshold = 15;
    public double kS = 0.1;
    private boolean twoMotors = false;

    public Lift(HardwareMap hardwareMap, boolean resetEncoder) {
        controller = new PIDFController(0.006, 0, 0.0001, 0);
        this.hardwareMap = hardwareMap;
        this.resetEncoder = resetEncoder;
        liftLeft = this.hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = this.hardwareMap.get(DcMotorEx.class, "liftRight");
        liftExtra = this.hardwareMap.get(DcMotorEx.class, "liftExtra");
        liftLeft.setDirection(DcMotorEx.Direction.REVERSE);
        liftRight.setDirection(DcMotorEx.Direction.FORWARD);
        liftExtra.setDirection(DcMotorSimple.Direction.REVERSE);

        if (resetEncoder) {
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftExtra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftExtra.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTwoMotors(boolean x) {
        twoMotors = x;
    }

    public void loop() {
        if (!manual) {
            int currentPos = liftLeft.getCurrentPosition();
            double pwr = controller.calculate(currentPos, target);
            if (Math.abs(target-currentPos) > errorThreshold) {
                pwr += kS * Math.signum(target - currentPos);
            }
//            double power = leftMotorPower.run(target - liftLeft.getCurrentPosition());
//            leftMotorPower.setDeadzone(10);
//            leftMotorPower.setHomedConstant(-.05);
//            if (liftLeft.getCurrentPosition() <= 10 && target == 0) {
//                leftMotorPower.setHomed(true);
//            } else {
//                leftMotorPower.setHomed(false);
//            }
            if (twoMotors) {
                liftLeft.setPower(pwr);
                liftRight.setPower(pwr);
            } else {
                liftExtra.setPower(pwr);
                liftRight.setPower(pwr);
                liftLeft.setPower(pwr);
            }
        }
        else {
            liftLeft.setPower(power);
            liftRight.setPower(power);
            liftExtra.setPower(power);
        }
    }
    public void setManual(boolean x) {
        manual = x;
    }


    public void setPower(double x) {
        power = x;
    }

    public void setTargetPos(double targetPos){
        target = targetPos;
    }

    public int getLiftPos() {
        return liftLeft.getCurrentPosition();
    }
}
