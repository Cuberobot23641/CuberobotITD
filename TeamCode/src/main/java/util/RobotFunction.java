package util;

import java.util.List;
import com.pedropathing.util.Timer;
public class RobotFunction {
    private final List<Runnable> functions;
    private final List<Double> executeTimes;
    private Timer timer;
    private int state = -1;
    private int index = 0;


    public RobotFunction(List<Runnable> functions, List<Double> executeTimes) {
        this.functions = functions;
        this.executeTimes = executeTimes;
        timer = new Timer();
    }


    // overloaded: one function + one wait time
    public RobotFunction(Runnable function, double executeTime) {
        this.functions = List.of(function);
        this.executeTimes = List.of(executeTime);
        timer = new Timer();
    }


    private void runState() {
        if (index > functions.size()-1) {
            setState(-1);
        } else if (timer.getElapsedTimeSeconds() > executeTimes.get(index)) {
            if (!(index+1 > functions.size()-1)) {
                functions.get(index+1).run();
            }
            timer.resetTimer();
            index++;
        }
    }




    public void run() {
        switch (state) {
            case 0:
                functions.get(state).run();
                setState(1);
                break;
            case 1:
                runState();
                break;
        }
    }


    public void start() {
        timer.resetTimer();
        index = 0;
        setState(0);
    }


    public void setState(int x) {
        state = x;
        timer.resetTimer();
    }


    public int getIndex() {
        return index;
    }


    public boolean isFinished() {
        return state == -1;
    }
}
