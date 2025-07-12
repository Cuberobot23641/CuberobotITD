package util;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SmartServo {
    private final Servo servo;
    private final Timer timer;
    private double currentPos;
    private double targetPos;
    // speed in servo units per second (0.0–1.0 range)
    private double speed;
    private double direction;

    public SmartServo(HardwareMap hardwareMap, String servoName, Servo.Direction direction, double servoRangeDegrees, double timeFor60Degrees) {
        servo = hardwareMap.get(Servo.class, servoName);
        servo.setDirection(direction);
        timer = new Timer();

        // Correct conversion to normalized units per second
        double degreesPerSecond = 60.0 / timeFor60Degrees;
        speed = degreesPerSecond / servoRangeDegrees;

        // Initialize current position from actual servo value
        currentPos = servo.getPosition();
        targetPos = currentPos;
    }

    public void loop() {
        double dt = timer.getElapsedTimeSeconds();
        double delta = speed * dt;

        if (currentPos != targetPos) {
            double remaining = targetPos - currentPos;

            // Avoid overshooting: clamp if within step
            if (Math.abs(remaining) <= delta) {
                currentPos = targetPos;
            } else {
                direction = Math.signum(remaining);
                currentPos += direction * delta;
            }
        }

        timer.resetTimer();
    }

    public double getCurrentPosition() {
        return currentPos;
    }

    public void setTarget(double tgt) {
        // Clamp to [0.0, 1.0] range
        tgt = Math.max(0.0, Math.min(1.0, tgt));

        targetPos = tgt;
        direction = Math.signum(targetPos - currentPos);
        servo.setPosition(targetPos); // Always command servo to move
    }

    public double getTarget() {
        return targetPos;
    }

    public boolean atTarget(double tolerance) {
        return Math.abs(currentPos - targetPos) <= tolerance;
    }

    public boolean atTarget() {
        return atTarget(1e-2); // Use a small epsilon
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }
}


