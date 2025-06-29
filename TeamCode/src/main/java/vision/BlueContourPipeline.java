package vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlueContourPipeline extends OpenCvPipeline {

    // Stores distances and angles of detected targets
    public static class TargetInfo {
        public final double distance;
        public final double angle;

        public TargetInfo(double distance, double angle) {
            this.distance = distance;
            this.angle = angle;
        }
    }

    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();
    private final List<MatOfPoint> contours = new ArrayList<>();

    private final Mat homographyMatrix = new Mat();
    private final List<TargetInfo> targets = new ArrayList<>();

    public BlueContourPipeline() {
        // Replace with your real homography matrix
        homographyMatrix.put(0, 0,
                0.05, 0.0, -10.0,  // H11, H12, H13
                0.0, 0.05, -10.0,  // H21, H22, H23
                0.0, 0.0, 1.0      // H31, H32, H33
        );
    }

    @Override
    public Mat processFrame(Mat input) {
        contours.clear();
        targets.clear();

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowerBlue = new Scalar(100, 150, 50);
        Scalar upperBlue = new Scalar(140, 255, 255);
        Core.inRange(hsv, lowerBlue, upperBlue, mask);

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) > 500) {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                Point center = rect.center;

                // Convert center to homogeneous coordinates
                Mat src = new Mat(3, 1, CvType.CV_64F);
                src.put(0, 0, center.x, center.y, 1.0);

                // Apply homography
                Mat dst = new Mat();
                Core.gemm(homographyMatrix, src, 1.0, new Mat(), 0.0, dst);

                double x = dst.get(0, 0)[0];
                double y = dst.get(1, 0)[0];
                double w = dst.get(2, 0)[0];

                double realX = x / w;
                double realY = y / w;

                double distance = Math.hypot(realX, realY);
                double angleDeg = Math.toDegrees(Math.atan2(realY, realX));

                targets.add(new TargetInfo(distance, angleDeg));
            }
        }

        return input;
    }

    // Returns list of all detected targets with their distance and angle
    public List<TargetInfo> getTargets() {
        return new ArrayList<>(targets); // return a copy for safety
    }

    // Returns the closest target, or null if none
    public TargetInfo getClosestTarget() {
        if (targets.isEmpty()) return null;
        TargetInfo closest = targets.get(0);
        for (TargetInfo t : targets) {
            if (t.distance < closest.distance) closest = t;
        }
        return closest;
    }

    // Returns the number of valid targets
    public int getTargetCount() {
        return targets.size();
    }
}



