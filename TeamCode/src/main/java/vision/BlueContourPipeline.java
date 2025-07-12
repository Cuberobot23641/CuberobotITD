package vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlueContourPipeline extends OpenCvPipeline {

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
    private final Mat mask1 = new Mat();
    private final Mat mask2 = new Mat();
    private final Mat hierarchy = new Mat();
    private final List<MatOfPoint> contours = new ArrayList<>();
    private final List<TargetInfo> targets = new ArrayList<>();

    private final Mat homographyMatrix = new Mat();

    public BlueContourPipeline() {
        // Replace with your calibrated matrix: pixel -> millimeters
        homographyMatrix.put(0, 0,
                1.62253078, -1.43247958, -516.481613,
                -0.03842739, -1.43247958, 571.965417,
                -0.000070355535, 0.00172971875, 1.0
        );
    }

    @Override
    public Mat processFrame(Mat input) {
        contours.clear();
        targets.clear();

        // Convert to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Red in HSV = two ranges
        Scalar lowerRed1 = new Scalar(0, 120, 70);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(170, 120, 70);
        Scalar upperRed2 = new Scalar(180, 255, 255);

        Core.inRange(hsv, lowerRed1, upperRed1, mask1);
        Core.inRange(hsv, lowerRed2, upperRed2, mask2);
        Core.add(mask1, mask2, mask);

        // Find contours in the red mask
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) > 500) {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                Point center = rect.center;

                // Convert pixel point to real-world mm using homography
                Mat src = new Mat(3, 1, CvType.CV_64F);
                src.put(0, 0, center.x, center.y, 1.0);

                Mat dst = new Mat();
                Core.gemm(homographyMatrix, src, 1.0, new Mat(), 0.0, dst);

                double x = dst.get(0, 0)[0];
                double y = dst.get(1, 0)[0];
                double w = dst.get(2, 0)[0];

                double realX = x / w;
                double realY = y / w;

                double distance = Math.hypot(realX, realY); // in mm
                double angleDeg = Math.toDegrees(Math.atan2(realY, realX));

                targets.add(new TargetInfo(distance, angleDeg));

                // Draw bounding box on original input for display
                Point[] boxPoints = new Point[4];
                rect.points(boxPoints);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }
            }
        }

        return input;
    }

    public List<TargetInfo> getTargets() {
        return new ArrayList<>(targets);
    }

    public TargetInfo getClosestTarget() {
        if (targets.isEmpty()) return null;
        TargetInfo closest = targets.get(0);
        for (TargetInfo t : targets) {
            if (t.distance < closest.distance) closest = t;
        }
        return closest;
    }

    public int getTargetCount() {
        return targets.size();
    }
}





