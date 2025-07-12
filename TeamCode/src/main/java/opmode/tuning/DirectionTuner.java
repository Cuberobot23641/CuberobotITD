package opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import util.SmartMotor;
import util.SmartServo;


@Config
@TeleOp(name="Example Direction Tuner")
public class DirectionTuner extends OpMode {
    // TODO: Copy and paste for all motors and servos
    public SmartMotor motor;
    public static double motorPower = 0;
    public static boolean motorForward = true;

    public SmartServo servo;
    public static double servoPosition = 0;
    public static boolean servoForward = true;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = new SmartMotor(hardwareMap, "liftLeft", DcMotorEx.Direction.FORWARD, true, false);
        servo = new SmartServo(hardwareMap, "clawIntake", Servo.Direction.FORWARD, 180, 0.15);
    }

    @Override
    public void loop() {
        if (motorForward) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        motor.setPower(motorPower);

        if (servoForward) {
            servo.setDirection(Servo.Direction.FORWARD);
        } else {
            servo.setDirection(Servo.Direction.REVERSE);
        }

        servo.setTarget(servoPosition);
        telemetry.update();
    }
}