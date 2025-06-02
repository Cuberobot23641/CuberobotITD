package util;

public class PositionCalculator {
    private static double turretLength = 7.3;
    public static double[] getPositions(double distX, double distY, double angle) {
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
}
