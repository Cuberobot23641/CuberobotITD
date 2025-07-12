package opmode.tuning;

public class PositionTuner {
    // the gist is pretty simple
    // essentially, you initialize a new robot but don't initialize its positions
    // the robot class will have an initPositions() method, so don't call that.

    // next, you have a boolean for each subsystem. if it is on, then it will move to its set targets
    // thats kinda it, loop at PositionTuning.java for more information
}
