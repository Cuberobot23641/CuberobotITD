package util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FiniteStateMachine {
    private List<State> states;
    private int state;
    private int index;

    public FiniteStateMachine(State...states) {
        this.states = new ArrayList<>();
        this.states.addAll(Arrays.asList(states));
        state = -1;
        index = 0;
    }

    public void start() {
        if (states != null) {
            state = 0;
            states.get(0).start();
        }
    }

    public void run() {
        if (state == 0) {
            if (states.get(index).isFinished()) {
                if (index < states.size() - 1) {
                    index++;
                    states.get(index).start();
                } else {
                    state = -1;
                }
            }
            for (State finiteState: states) {
                finiteState.run();
            }
        }
    }

    public boolean isFinished() {
        return state == -1;
    }
}
