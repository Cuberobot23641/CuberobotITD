package util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class State {
    private List<Condition> conditions;
    private Runnable runnable;
    private int state;

    public State() {
        this.conditions = new ArrayList<>();
        state = -1;
    }

    public void start() {
        for (Condition condition: conditions) {
            condition.start();
        }
        state = 0;
    }

    public void run() {
        for (Condition condition: conditions) {
            if (!condition.isFinished()) {
                return;
            }
        }
        if (runnable != null) {
            runnable.run();
        }
        state = -1;
    }

    public State then(Runnable runnable) {
        this.runnable = runnable;
        return this;
    }

    public State addCondition(Condition condition) {
        conditions.add(condition);
        return this;
    }

    public boolean isFinished() {
        return state == -1;
    }
}
