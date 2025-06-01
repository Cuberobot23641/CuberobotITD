package robot;

public final class RobotConstants {
    private RobotConstants() {}

    // lift
    public static final int LIFT_SPEC_GRAB = 0;
    public static final int LIFT_SPEC_SCORE = 1260; //980 SHOULD BE 1020
    public static final int LIFT_SAMPLE_HIGH = 2200;
    public static final int LIFT_SAMPLE_LOW = 1000;
    public static final int LIFT_TRANSFER = 0;

    // extension
    public static final int EXTENSION_MAX = 600;
    public static final int EXTENSION_MID = 300;
    public static final int EXTENSION_MIN = 0;
    public static final int EXTENSION_TRANSFER = 300;

    // deposit
    public static final double DEPOSIT_CLAW_OPEN = 0.05;
    public static final double DEPOSIT_CLAW_CLOSED = 0.37;
    public static final double DEPOSIT_ELBOW_TRANSFER = 0.84;//0.91;
    public static final double DEPOSIT_ELBOW_SPEC_GRAB = 0.1;
    public static final double DEPOSIT_ELBOW_SPEC_SCORE = 0.8;//0.79; // 0.74 SHOULD BE 0.75
    public static final double DEPOSIT_ELBOW_SAMPLE_SCORE = 0.4;
    public static final double DEPOSIT_LINKAGE_EXTEND = 0.15;
    public static final double DEPOSIT_LINKAGE_RETRACT = 0.15;

    // intake
    public static final double INTAKE_CLAW_OPEN = 0.24;
    public static final double INTAKE_CLAW_CLOSED = 0.58;
    public static final double INTAKE_ELBOW_DEFAULT = 0.25;
    public static final double INTAKE_ELBOW_TRANSFER = 0.85;
    public static final double INTAKE_ELBOW_DOWN = 0.145;
    // good for not getting in the way, use this in the middle of retract
    public static final double INTAKE_ELBOW_IN = 0.25;
    public static final double INTAKE_ELBOW_HOVER = 0.2;
    public static final double INTAKE_ELBOW_DROP_OFF = 0.25;
    // public static final double INTAKE_ELBOW_IN = 0.6;
    public static final double INTAKE_WRIST_DEFAULT = 0.5;
    public static final double INTAKE_TURRET_DEFAULT = 0.52;
    public static final double INTAKE_TURRET_DROP_OFF = 0.0;
    public static final double INTAKE_WRIST_DROP_OFF = 0.2;
    public static final double INTAKE_TURRET_TRANSFER = 0.52;
}
