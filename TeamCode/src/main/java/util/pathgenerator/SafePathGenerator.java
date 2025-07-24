package util.pathgenerator;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import util.pathgenerator.Point2D;


public class SafePathGenerator {
    private PathBuilder pathBuilder;
    private PathFinder pathFinder;
    private List<PathChain> paths;
    private List<TargetPose> points;
    public SafePathGenerator(TargetPose...points) {
        pathFinder = new PathFinder(120, 120, 8);
        // as an example
        pathFinder.addObstacleRect(10, 10, 30, 30);
        pathFinder.inflateObstacles();

        pathBuilder = new PathBuilder();
        paths = new ArrayList<>();
        this.points = new ArrayList<>();
        this.points.addAll(Arrays.asList(points));
    }

    // call this method in init(), then you can use the paths in paths
    public void generatePaths() {
        TargetPose currentPose = null;
        for (TargetPose point: points) {
            if (currentPose == null) {
                currentPose = point;
            } else {
                Point2D.Double[] cpts = pathFinder.aStarSearch((int) currentPose.getX(), (int) currentPose.getY(), (int) point.getX(), (int) point.getY());
                ArrayList<Point> allPoints = new ArrayList<>();
                for (Point2D.Double pt : cpts) {
                    allPoints.add(new Point(pt.getX(), pt.getY(), Point.CARTESIAN));
                }
                PathChain path = pathBuilder
                        .addPath(
                                new BezierCurve(
                                        allPoints
                                )
                        )
                        .setLinearHeadingInterpolation(currentPose.getHeading(), point.getHeading())
                        .build();
                paths.add(path);
                currentPose = point;
            }
        }
    }
    // when following a path just call this
    public List<PathChain> getPaths() {
        return paths;
    }
}
