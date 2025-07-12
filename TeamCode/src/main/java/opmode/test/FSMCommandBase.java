package opmode.test;

import robot.robots.AutonomousRobot;
import util.fsm.State;
import util.fsm.StateMachine;
import util.fsm.Transition;

public class FSMCommandBase {
    private AutonomousRobot robot;
    public StateMachine stateMachine, grabSample;
    public FSMCommandBase(AutonomousRobot robot) {
        this.robot = robot;

        stateMachine = new StateMachine(
                new State("state 1")
                        .minTime(100)
                        .maxTime(600)
                        .onExit(() -> robot.deposit.openDepositClaw())
                        .transition(new Transition(() -> robot.extension.getExtensionPos() > 300, "state 2")),
                new State("state 2")
                        .transition(new Transition(() -> robot.lift.getLiftPos() > 300))
                        .maxTime(1000)
                        .fallbackState("state 1")
                        .onEnter(() -> robot.lift.setTargetPos(400)),
                new State()
                        .onExit(() -> System.out.println("done"))
                        .maxTime(69420)
        );

//        grabSample = new StateMachine(
//                new State()
//                        .onEnter(() -> {
//                            robot.extension.setTargetPos(400);
//                            robot.intake.setElbowIntakePos(0.3);
//                            robot.intake.setTurretPos(0.5);
//                            robot.intake.setWristPos(0.3);
//                        })
//                        .transition(() -> robot.extension.atTarget() && robot.intake.atTargets()),
//                new State()
//                        .onEnter(() -> robot.intake.setElbowIntakePos(0.2))
//                        .transition(() -> robot.intake.atTargets()),
//                new State()
//                        .onEnter(() -> robot.intake.closeIntakeClaw())
//                        .transition(() -> robot.intake.atTargets()),
//                new State()
//                        .onEnter(() -> {
//                            robot.extension.setTargetPos(0);
//                            robot.intake.setElbowIntakePos(0.3);
//                            robot.intake.setTurretPos(0.5);
//                            robot.intake.setWristPos(0.3);
//                        })
//                        .transition(() -> robot.extension.atTarget() && robot.intake.atTargets())
//        );
    }

    public void update() { // should we name all subsystems and robot class loop methods to periodic?
        stateMachine.run();
    }

}
