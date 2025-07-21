package util;

public class VoltageScaler {
    public static double calculateVoltageScaler(double nominalVoltage, double actualVoltage, double frictionConstant) {
        return (nominalVoltage - (nominalVoltage * frictionConstant)) / (actualVoltage - ((Math.pow(nominalVoltage, 2) / actualVoltage) * frictionConstant));
    }
}
