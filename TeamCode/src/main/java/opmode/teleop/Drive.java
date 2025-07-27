package opmode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import robot.robots.TeleOpRobot;

@TeleOp(name="drive no heading lock")
public class Drive extends OpMode {
    TeleOpRobot robot;

    @Override
    public void init() {
        robot = new TeleOpRobot(hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.loop();
        robot.updateControls();
    }
}
