package robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

public class Light {
    private final Servo servoLight;

    public Light(HardwareMap hardwareMap) {
        servoLight = hardwareMap.get(Servo.class, "servoLight");
        PwmControl light = (PwmControl) servoLight;
        light.setPwmEnable();
        light.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void on() {
        servoLight.setPosition(1);
    }

    public void off() {
        servoLight.setPosition(0);
    }
}
