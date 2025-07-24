package util.pathgenerator;

import java.util.*;

public class PathFinder {
    private final int ROW;
    private final int COL;
    private final int ROBOT_RADIUS_CELLS;
    // Directions: N, S, E, W, NE, NW, SE, SW
    private final int[] rowNum = {-1, 1, 0, 0, -1, -1, 1, 1};
    private final int[] colNum = {0, 0, 1, -1, 1, -1, 1, -1};
    private int[][] grid;

    public PathFinder(int numRows, int numCols, int robotRadius) {
        ROW = numRows;
        COL = numCols;
        ROBOT_RADIUS_CELLS = robotRadius;
        grid = new int[ROW][COL];
        for (int[] row : grid)
            Arrays.fill(row, 1);
    }
    private boolean isValid(int row, int col) {
        return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
    }

    private boolean isUnBlocked(int row, int col) {
        return grid[row][col] == 1;
    }

    private boolean isDestination(int row, int col, int destRow, int destCol) {
        return row == destRow && col == destCol;
    }

    private double calculateHValue(int row, int col, int destRow, int destCol) {
        return Math.sqrt((row - destRow) * (row - destRow) + (col - destCol) * (col - destCol));
    }

    private List<Point2D.Double> tracePath(Cell[][] cellDetails, int destRow, int destCol) {
        System.out.println("The path is:");
        int row = destRow;
        int col = destCol;

        Stack<Point2D.Double> path = new Stack<>();
        while (!(cellDetails[row][col].parentRow == row && cellDetails[row][col].parentCol == col)) {
            path.push(new Point2D.Double(row, col));
            int tempRow = cellDetails[row][col].parentRow;
            int tempCol = cellDetails[row][col].parentCol;
            row = tempRow;
            col = tempCol;
        }
        path.push(new Point2D.Double(row, col));

        List<Point2D.Double> pathList = new ArrayList<>();
        while (!path.isEmpty()) {
            Point2D.Double p = path.pop();
            System.out.print("-> (" + p.x + ", " + p.y + ") ");
            pathList.add(p);
        }
        System.out.println();
        return pathList;
    }

    public void inflateObstacles() {
        int[][] inflated = new int[ROW][COL];
        for (int i = 0; i < ROW; i++) {
            for (int j = 0; j < COL; j++) {
                inflated[i][j] = grid[i][j];
            }
        }

        for (int i = 0; i < ROW; i++) {
            for (int j = 0; j < COL; j++) {
                if (grid[i][j] == 0) {
                    for (int di = -ROBOT_RADIUS_CELLS; di <= ROBOT_RADIUS_CELLS; di++) {
                        for (int dj = -ROBOT_RADIUS_CELLS; dj <= ROBOT_RADIUS_CELLS; dj++) {
                            int ni = i + di;
                            int nj = j + dj;
                            if (isValid(ni, nj)) {
                                inflated[ni][nj] = 0;
                            }
                        }
                    }
                }
            }
        }
        grid = inflated.clone();
    }

    public PathFinder addObstacleRect(int startRow, int startCol, int endRow, int endCol) {
        for (int i = Math.max(0, startRow); i <= Math.min(ROW - 1, endRow); i++) {
            for (int j = Math.max(0, startCol); j <= Math.min(COL - 1, endCol); j++) {
                grid[i][j] = 0;
            }
        }
        return this;
    }

    public Point2D.Double[] aStarSearch(int startRow, int startCol, int destRow, int destCol) {
        if (!isValid(startRow, startCol) || !isValid(destRow, destCol)) {
            System.out.println("Invalid start or destination");
            return null;
        }

        if (!isUnBlocked(startRow, startCol) || !isUnBlocked(destRow, destCol)) {
            System.out.println("Start or destination is blocked");
            return null;
        }

        if (isDestination(startRow, startCol, destRow, destCol)) {
            System.out.println("We are already at the destination");
            return null;
        }

        boolean[][] closedList = new boolean[ROW][COL];
        Cell[][] cellDetails = new Cell[ROW][COL];

        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                cellDetails[i][j] = new Cell();

        int i = startRow, j = startCol;
        cellDetails[i][j].f = 0.0;
        cellDetails[i][j].g = 0.0;
        cellDetails[i][j].h = 0.0;
        cellDetails[i][j].parentRow = i;
        cellDetails[i][j].parentCol = j;

        PriorityQueue<Pair> openList = new PriorityQueue<>(Comparator.comparingDouble(p -> p.f));
        openList.add(new Pair(0.0, i, j));

        boolean foundDest = false;

        while (!openList.isEmpty()) {
            Pair p = openList.poll();
            i = p.row;
            j = p.col;
            closedList[i][j] = true;

            for (int dir = 0; dir < 8; dir++) {
                int newRow = i + rowNum[dir];
                int newCol = j + colNum[dir];

                if (isValid(newRow, newCol)) {
                    if (isDestination(newRow, newCol, destRow, destCol)) {
                        cellDetails[newRow][newCol].parentRow = i;
                        cellDetails[newRow][newCol].parentCol = j;
                        System.out.println("Destination found");
                        List<Point2D.Double> path = tracePath(cellDetails, destRow, destCol);
                        Point2D.Double[] bezier = fitCubicBezier(path);
                        printControlPoints(bezier);
                        foundDest = true;
                        return bezier;
                    } else if (!closedList[newRow][newCol] && isUnBlocked(newRow, newCol)) {
                        double gNew = cellDetails[i][j].g + 1.0;
                        double hNew = calculateHValue(newRow, newCol, destRow, destCol);
                        double fNew = gNew + hNew;

                        if (cellDetails[newRow][newCol].f == Double.MAX_VALUE || cellDetails[newRow][newCol].f > fNew) {
                            openList.add(new Pair(fNew, newRow, newCol));
                            cellDetails[newRow][newCol].f = fNew;
                            cellDetails[newRow][newCol].g = gNew;
                            cellDetails[newRow][newCol].h = hNew;
                            cellDetails[newRow][newCol].parentRow = i;
                            cellDetails[newRow][newCol].parentCol = j;
                        }
                    }
                }
            }
        }

        if (!foundDest) {
            System.out.println("Failed to find the Destination Cell");
        }
        return null;
    }

    // --- Bezier fitting methods ---

    private double[] chordLengthParameterize(List<Point2D.Double> points) {
        double[] distances = new double[points.size()];
        distances[0] = 0.0;
        for (int i = 1; i < points.size(); i++) {
            distances[i] = distances[i - 1] + points.get(i).distance(points.get(i - 1));
        }
        double total = distances[distances.length - 1];
        for (int i = 0; i < distances.length; i++) {
            distances[i] /= total;
        }
        return distances;
    }

    private Point2D.Double[] fitCubicBezier(List<Point2D.Double> points) {
        int n = points.size();
        Point2D.Double B0 = points.get(0);
        Point2D.Double B3 = points.get(n - 1);
        double[] t = chordLengthParameterize(points);

        double[][] A = new double[n][2];
        double[][] C = new double[n][2];

        for (int i = 0; i < n; i++) {
            double ti = t[i];
            double b = 1 - ti;
            double b0 = b * b * b;
            double b1 = 3 * ti * b * b;
            double b2 = 3 * ti * ti * b;
            double b3 = ti * ti * ti;

            A[i][0] = b1;
            A[i][1] = b2;

            C[i][0] = points.get(i).x - (b0 * B0.x + b3 * B3.x);
            C[i][1] = points.get(i).y - (b0 * B0.y + b3 * B3.y);
        }

        double[][] ATA = new double[2][2];
        double[] ATC_x = new double[2];
        double[] ATC_y = new double[2];

        for (int i = 0; i < n; i++) {
            ATA[0][0] += A[i][0] * A[i][0];
            ATA[0][1] += A[i][0] * A[i][1];
            ATA[1][0] += A[i][1] * A[i][0];
            ATA[1][1] += A[i][1] * A[i][1];

            ATC_x[0] += A[i][0] * C[i][0];
            ATC_x[1] += A[i][1] * C[i][0];

            ATC_y[0] += A[i][0] * C[i][1];
            ATC_y[1] += A[i][1] * C[i][1];
        }

        double det = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0];
        if (Math.abs(det) < 1e-12) {
            // fallback to midpoint control points
            Point2D.Double mid = midpoint(B0, B3);
            return new Point2D.Double[]{B0, mid, mid, B3};
        }
        double invDet = 1.0 / det;

        double B1x = invDet * (ATA[1][1] * ATC_x[0] - ATA[0][1] * ATC_x[1]);
        double B2x = invDet * (-ATA[1][0] * ATC_x[0] + ATA[0][0] * ATC_x[1]);

        double B1y = invDet * (ATA[1][1] * ATC_y[0] - ATA[0][1] * ATC_y[1]);
        double B2y = invDet * (-ATA[1][0] * ATC_y[0] + ATA[0][0] * ATC_y[1]);

        Point2D.Double B1 = new Point2D.Double(B1x, B1y);
        Point2D.Double B2 = new Point2D.Double(B2x, B2y);

        return new Point2D.Double[]{B0, B1, B2, B3};
    }

    private Point2D.Double midpoint(Point2D.Double p1, Point2D.Double p2) {
        return new Point2D.Double((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
    }

    private void printControlPoints(Point2D.Double[] controlPoints) {
        System.out.println("\nBezier Control Points:");
        for (int i = 0; i < controlPoints.length; i++) {
            System.out.printf("B%d: (%.4f, %.4f)%n", i, controlPoints[i].x, controlPoints[i].y);
        }
    }
}
