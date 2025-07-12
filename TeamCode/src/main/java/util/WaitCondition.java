package util;

import com.pedropathing.util.Timer;

public class WaitCondition extends Condition {
    private int state;
    private final Timer timer;
    private final int waitTimeMs;
    public WaitCondition(int waitTimeMs) {
        timer = new Timer();
        this.waitTimeMs = waitTimeMs;
        state = -1;
    }

    @Override
    public void start() {
        timer.resetTimer();
        state = 0;
    }

    @Override
    public void run() {
        if (state == 0) {
            if (timer.getElapsedTime() >= waitTimeMs) {
                state = -1;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return state == -1;
    }
}
