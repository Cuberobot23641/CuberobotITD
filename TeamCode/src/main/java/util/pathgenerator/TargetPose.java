package util.pathgenerator;

public class TargetPose extends ControlPoint {
    private double x;
    private double y;
    private double heading;
    public TargetPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}
