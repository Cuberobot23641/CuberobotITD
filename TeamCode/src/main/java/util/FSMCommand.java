package util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FSMCommand {
    private List<FSMCondition> conditions;
    private List<Runnable> runnables;
    private int state;

    public FSMCommand(FSMCondition...conditions) {
        this.conditions = new ArrayList<>();
        this.conditions.addAll(Arrays.asList(conditions));
        state = -1;
    }

    public FSMCommand(Runnable...runnables) {
        this.runnables = new ArrayList<>();
        this.runnables.addAll(Arrays.asList(runnables));
        state = -1;
    }


    public void start() {
        if (conditions != null) {
            for (FSMCondition condition: conditions) {
                condition.start();
            }
            state = 0;
        } else {
            for (Runnable runnable: runnables) {
                runnable.run();
            }
            state = -1;
        }
    }

    public void run() {
        if (conditions != null) {
            for (FSMCondition condition: conditions) {
                if (!condition.isFinished()) {
                    return;
                }
            }
            state = -1;
        }
    }

    public boolean isFinished() {
        return state == -1;
    }
}
