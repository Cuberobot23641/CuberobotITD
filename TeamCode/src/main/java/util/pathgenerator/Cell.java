package util.pathgenerator;

public class Cell {
    int parentRow, parentCol;
    double f, g, h;

    public Cell() {
        f = g = h = Double.MAX_VALUE;
        parentRow = parentCol = -1;
    }
}
