package util;
// FSMCommand
// takes in a list of FSMconditions OR takes in a list of runnables
// FSMWait: waits a certain amount of time
// FSMCondition: waits for a condition to be true

// FSMConditions run until all are true
// start will reset timer

// TODO: in theory we could use this for auto...
// have a FSMCommand: () -> follower.followPath() and () -> robot.something.start()
// FSMCommand with condition: !follower.isBusy()
// and so on and so forth

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FSM {
    private List<FSMCommand> commands;
    private int state;
    private int index;

    public FSM(FSMCommand...commands) {
        this.commands = new ArrayList<>();
        this.commands.addAll(Arrays.asList(commands));
        state = -1;
        index = 0;
    }

    public void start() {
        if (commands != null) {
            state = 0;
            commands.get(0).start();
        }
    }

    public void run() {
        if (state == 0) {
            if (commands.get(index).isFinished()) {
                if (index < commands.size() - 1) {
                    index++;
                    commands.get(index).start();
                } else {
                    state = -1;
                }
            }
            for (FSMCommand command: commands) {
                command.run();
            }
        }
    }

    public boolean isFinished() {
        return state == -1;
    }
}
