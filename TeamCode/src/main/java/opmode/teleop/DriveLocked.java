package opmode.teleop;


import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import robot.robots.TeleOpRobot;
import robot.robots.TeleOpRobotV2;

@TeleOp(name="drive with heading lock???")
public class DriveLocked extends OpMode {
    TeleOpRobotV2 robot;

    @Override
    public void init() {
        robot = new TeleOpRobotV2(hardwareMap, gamepad1, gamepad2);
        try {
            sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.initSubsystems();
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.loop();
        robot.updateControls();
        telemetry.addData("current heading", robot.drivetrain.getCurrentHeading());
        telemetry.addData("target heading", robot.drivetrain.getTargetHeading());
        telemetry.update();
    }
}
