package util;

import java.util.function.Supplier;

public class Condition {
    private int state;
    private Supplier<Boolean> condition;
    public Condition(Supplier<Boolean> condition) {
        this.condition = condition;
        state = -1;
    }

    public Condition() {}

    public void start() {
        state = 0;
    }

    public void run() {
        if (state == 0) {
            if (condition.get()) {
                state = -1;
            }
        }
    }

    public boolean isFinished() {
        return state == -1;
    }
}
