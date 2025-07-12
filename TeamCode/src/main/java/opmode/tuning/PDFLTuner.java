package opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import util.PDFLController;
import util.SmartMotor;

@Config
@TeleOp(name="Example PDFL Tuner")
public class PDFLTuner extends OpMode {
    private PDFLController controller;
    public static double p = 0, d = 0, f = 0, l = 0, maxPower = 1, tolerance = 0;
    public static int target = 0;

    public SmartMotor motor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PDFLController(p, d, f, l, maxPower, tolerance);
        motor = new SmartMotor(hardwareMap, "liftLeft", DcMotorEx.Direction.REVERSE, true, false);
    }

    @Override
    public void loop() {
        double currentPos = motor.getCurrentPosition();
        controller.setCoefficients(p, d, f, l, maxPower, tolerance);
        controller.setTarget(target);
        double power = controller.calculate(currentPos);

        motor.setPower(power);

        telemetry.addData("current pos", currentPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
