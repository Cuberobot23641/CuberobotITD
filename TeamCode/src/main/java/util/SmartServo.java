package util;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SmartServo {
    private final Servo servo;
    private final Timer timer;
    private double currentPos;
    private double targetPos;
    // speed: servo ticks / time
    private double speed;
    private double direction;

    public SmartServo(HardwareMap hardwareMap, String servoName, Servo.Direction direction, double servoRangeDegrees, double timeFor60Degrees) {
        servo = hardwareMap.get(Servo.class, servoName);
        servo.setDirection(direction);
        timer = new Timer();
        speed = (60 / (servoRangeDegrees * timeFor60Degrees));
        currentPos = 0;
        targetPos = 0;
    }

    public void loop() {
        double dt = timer.getElapsedTimeSeconds();
        if (currentPos != targetPos) {
            currentPos += speed * direction * dt;
            if (direction <= 0 && currentPos <= targetPos) {
                currentPos = targetPos;
            }
            if (direction > 0 && currentPos >= targetPos) {
                currentPos = targetPos;
            }
        }
        timer.resetTimer();
    }
    public double getCurrentPosition() {
        return currentPos;
    }
    public void setTarget(double tgt) {
        direction = Math.signum((tgt - currentPos));
        servo.setPosition(tgt);
        targetPos = tgt;
    }
    public double getTarget() {
        return targetPos;
    }
    public boolean atTarget(double tolerance) {
        return Math.abs((currentPos - targetPos)) <= tolerance;
    }

    public boolean atTarget() {
        return currentPos == targetPos;
    }
}

