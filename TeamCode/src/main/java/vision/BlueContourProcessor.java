package vision;


import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;

import java.util.ArrayList;
import java.util.List;

public class BlueContourProcessor implements VisionProcessor {

    public static class TargetInfo {
        public final double xDistance;
        public final double yDistance;
        public final double angle;

        public TargetInfo(double xDistance, double yDistance, double angle) {
            this.xDistance = xDistance;
            this.yDistance = yDistance;
            this.angle = angle;
        }
    }

    public double calculateAngle(RotatedRect rect) {
        // TODO: implementation
        return rect.angle;
        // if (rect.)
    }

    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat mask1 = new Mat();
    private final Mat mask2 = new Mat();
    private final Mat hierarchy = new Mat();

    private final List<MatOfPoint> contours = new ArrayList<>();
    private final List<TargetInfo> targets = new ArrayList<>();

    private final Mat homographyMatrix = new Mat();
    private final Paint greenPaint = new Paint();

    {
        greenPaint.setColor(Color.GREEN);
        greenPaint.setStrokeWidth(2);
        greenPaint.setStyle(Paint.Style.STROKE);
    }

    public BlueContourProcessor() {
        homographyMatrix.create(3, 3, CvType.CV_64F);
        homographyMatrix.put(0, 0,
                1.62253078, -1.43247958, -516.481613,
                -0.03842739, -1.43247958, 571.965417,
                -0.000070355535, 0.00172971875, 1.0
        );
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Calibration can be used if needed
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        contours.clear();
        targets.clear();

        Core.rotate(input, input, Core.ROTATE_180);

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowerRed1 = new Scalar(0, 120, 70);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(170, 120, 70);
        Scalar upperRed2 = new Scalar(180, 255, 255);

        Core.inRange(hsv, lowerRed1, upperRed1, mask1);
        Core.inRange(hsv, lowerRed2, upperRed2, mask2);
        Core.add(mask1, mask2, mask);

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) > 100) {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                Point center = rect.center;

                // Apply homography to get real-world coordinates
                Mat src = new Mat(3, 1, CvType.CV_64F);
                src.put(0, 0, center.x, center.y, 1.0);

                Mat dst = new Mat();
                Core.gemm(homographyMatrix, src, 1.0, new Mat(), 0.0, dst);

                double x = dst.get(0, 0)[0];
                double y = dst.get(1, 0)[0];
                double w = dst.get(2, 0)[0];

                double realX = x / w;
                double realY = y / w;

                double angleDeg = calculateAngle(rect);

                TargetInfo t = new TargetInfo(realX, realY, angleDeg);
//                if isValid(t) {
//
//                }

                targets.add(t);

                // Draw bounding box on the original input
                Point[] boxPoints = new Point[4];
                rect.points(boxPoints);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }
            }
        }

        return null; // Output shown via camera stream, not used here
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) > 100) {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                Point[] boxPoints = new Point[4];
                rect.points(boxPoints);

                for (int i = 0; i < 4; i++) {
                    Point p1 = boxPoints[i];
                    Point p2 = boxPoints[(i + 1) % 4];
                    canvas.drawLine(
                            (float) (p1.x * scaleBmpPxToCanvasPx),
                            (float) (p1.y * scaleBmpPxToCanvasPx),
                            (float) (p2.x * scaleBmpPxToCanvasPx),
                            (float) (p2.y * scaleBmpPxToCanvasPx),
                            greenPaint
                    );
                }
            }
        }
    }

    public List<TargetInfo> getTargets() {
        return new ArrayList<>(targets);
    }

    private boolean isValid(TargetInfo ti) {
        // wait till we do inches first
        if (Math.hypot(ti.xDistance, ti.yDistance) < 7.3) {
            return false;
        }
        if (Math.abs(ti.xDistance) > 7.3 || Math.abs(ti.yDistance) > 27) {
            return false;
        }
        return true;
    }

    public TargetInfo getClosestTarget() {
        if (targets.isEmpty()) return null;
        TargetInfo closest = targets.get(0);
        // sort by y for now
        for (TargetInfo t : targets) {
            if (t.yDistance < closest.yDistance) closest = t;
        }
        return closest;
    }

    public int getTargetCount() {
        return targets.size();
    }
}