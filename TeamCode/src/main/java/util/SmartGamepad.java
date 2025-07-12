package util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class SmartGamepad {
    private Gamepad gamepad;

    // Previous state for edge detection
    private boolean aPrev, bPrev, xPrev, yPrev;
    private boolean leftBumperPrev, rightBumperPrev;
    private boolean dpadUpPrev, dpadDownPrev, dpadLeftPrev, dpadRightPrev;
    private double rightTriggerPrev, leftTriggerPrev;
    private double triggerThreshold = 0.05;

    public SmartGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    // Call this once per loop to update internal state
    public void update() {
        aPrev = gamepad.a;
        bPrev = gamepad.b;
        xPrev = gamepad.x;
        yPrev = gamepad.y;
        leftBumperPrev = gamepad.left_bumper;
        rightBumperPrev = gamepad.right_bumper;
        dpadUpPrev = gamepad.dpad_up;
        dpadDownPrev = gamepad.dpad_down;
        dpadLeftPrev = gamepad.dpad_left;
        dpadRightPrev = gamepad.dpad_right;
        rightTriggerPrev = gamepad.right_trigger;
        leftTriggerPrev = gamepad.left_trigger;
    }

    // Edge detection for buttons
    public boolean aPressed() {
        return gamepad.a && !aPrev;
    }

    public boolean bPressed() {
        return gamepad.b && !bPrev;
    }

    public boolean xPressed() {
        return gamepad.x && !xPrev;
    }

    public boolean yPressed() {
        return gamepad.y && !yPrev;
    }

    public boolean leftBumperPressed() {
        return gamepad.left_bumper && !leftBumperPrev;
    }

    public boolean rightBumperPressed() {
        return gamepad.right_bumper && !rightBumperPrev;
    }

    public boolean dpadUpPressed() {
        return gamepad.dpad_up && !dpadUpPrev;
    }

    public boolean dpadDownPressed() {
        return gamepad.dpad_down && !dpadDownPrev;
    }

    public boolean dpadLeftPressed() {
        return gamepad.dpad_left && !dpadLeftPrev;
    }

    public boolean dpadRightPressed() {
        return gamepad.dpad_right && !dpadRightPrev;
    }

    public boolean leftTriggerPressed() {
        return gamepad.left_trigger > triggerThreshold && !(leftTriggerPrev > triggerThreshold);
    }

    public boolean rightTriggerPressed() {
        return gamepad.right_trigger > triggerThreshold && !(rightTriggerPrev > triggerThreshold);
    }

    // Analog stick accessors
    public float getLeftStickX() { return gamepad.left_stick_x; }
    public float getLeftStickY() { return gamepad.left_stick_y; }
    public float getRightStickX() { return gamepad.right_stick_x; }
    public float getRightStickY() { return gamepad.right_stick_y; }

    public float getLeftTrigger() { return gamepad.left_trigger; }
    public float getRightTrigger() { return gamepad.right_trigger; }

    // Current state passthrough (optional)
    public boolean a() { return gamepad.a; }
    public boolean b() { return gamepad.b; }
    public boolean x() { return gamepad.x; }
    public boolean y() { return gamepad.y; }
}
