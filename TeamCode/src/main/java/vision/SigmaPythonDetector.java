package vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public class SigmaPythonDetector {

    // TODO: we want to change the sorting algorithm so that it grabs a sample that isn't close to others
    // Limelight and claw configuration
    public static double turretLength = 7.3;
    private Limelight3A limelight;
    private String targetColor = "blue sample";
    private double distX = 0;
    private double distY = 0;

    private double angle = 0;

    private double distX2 = 0;
    private double distY2 = 0;
    private double angle2 = 0;
    private long staleness = 0;
    private boolean isValid = false;

    public SigmaPythonDetector(HardwareMap hardwareMap, String targetColor) {
        this.targetColor = targetColor;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (this.targetColor.equals("blue sample")) {
            limelight.pipelineSwitch(5);
        } else if (this.targetColor.equals("red sample")){
            limelight.pipelineSwitch(6);
        } else {
            limelight.pipelineSwitch(7);
        }
        System.out.println("started");
        limelight.start();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            System.out.println("null result");
        }
        if (result != null) {
            //if (result.isValid()) {
            staleness = result.getStaleness();
            double[] pythonOutput = result.getPythonOutput();
//            System.out.println(pythonOutput);
//            System.out.println(pythonOutput[0]);
//            System.out.println(pythonOutput[1]);
//            System.out.println(pythonOutput[2]);
            distX = pythonOutput[0];
            distY = pythonOutput[1];
            angle = pythonOutput[2];

            distX2 = pythonOutput[3];
            distY2 = pythonOutput[4];
            angle2 = pythonOutput[5];
//            if (pythonOutput[0] != 0.0 && pythonOutput[1] != 0.0 && pythonOutput) {
//                System.out.println("new result");
//                lastOneValid = true;
//            } else {
//                System.out.println("No new result");
//                lastOneValid = false;
//            }
        }
    }
    public double[] getPositions() {
        if (distX == 0 && distY == 0 && angle == 0) {
            return new double[] {0, 0, 0};
        }
        double turretAngle = 0;
        double turretExtension = 0;
        if (Math.abs(distX) > turretLength) {
            turretAngle = Math.asin(Math.signum(distX));
        } else {
            turretAngle = Math.toDegrees(Math.asin(distX / turretLength));
        }

        if (Math.abs(distX) > turretLength) {
            turretExtension = 0;
        } else {
            turretExtension = Math.sqrt(Math.abs(Math.pow(turretLength, 2) - Math.pow(distX, 2)));
        }
        //  add the angles -turretAngle and sampleAngle, then convert from -90 to 90
        double wristAngle = -turretAngle + angle;
        if (wristAngle < -90) {
            wristAngle += 180;
        } else if (wristAngle > 90) {
            wristAngle -= 180;
        }

        double extensionDistance = distY - turretExtension;
        return new double[] {extensionDistance, turretAngle, wristAngle};
    }

    public double[] getPositions2() {
        if (distX2 == 0 && distY2 == 0 && angle2 == 0) {
            return new double[] {0, 0, 0};
        }
        double turretAngle = 0;
        double turretExtension = 0;
        if (Math.abs(distX2) > turretLength) {
            turretAngle = Math.asin(Math.signum(distX2));
        } else {
            turretAngle = Math.toDegrees(Math.asin(distX2 / turretLength));
        }

        if (Math.abs(distX2) > turretLength) {
            turretExtension = 0;
        } else {
            turretExtension = Math.sqrt(Math.abs(Math.pow(turretLength, 2) - Math.pow(distX2, 2)));
        }
        //  add the angles -turretAngle and sampleAngle, then convert from -90 to 90
        double wristAngle = -turretAngle + angle2;
        if (wristAngle < -90) {
            wristAngle += 180;
        } else if (wristAngle > 90) {
            wristAngle -= 180;
        }

        double extensionDistance = distY2 - turretExtension;
        return new double[] {extensionDistance, turretAngle, wristAngle};
    }

    public double[] getDistances() {
        return new double[] {distX, distY, angle};
    }

    public double[] getDistances2() {
        return new double[] {distX2, distY2, angle2};
    }

    public boolean isFresh() {
        return staleness < 100;
    }

    public void off() {
        limelight.stop();
    }

    public void on() {
        limelight.start();
        // TODO: check maybe this will be good idk
        limelight.setPollRateHz(30);
    }

    public boolean isSampleThere() {
        return isValid;
    }

    public boolean isTwoSampleThere() {
        return isValid;
    }
    // use this to change the pipeline to the blue pipeline that selects the farthest right one
    public void switchPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public void idk() {
        limelight.setPollRateHz(30);
        //limelight.
    }
    public double lastUpdate() {
        return limelight.getTimeSinceLastUpdate();
    }
}
