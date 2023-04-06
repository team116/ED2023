package frc.robot.autos.primitives;

import frc.robot.commands.BaseArmCommand;
import frc.robot.subsystems.Arm;

public class HoldArmCommand extends BaseArmCommand {
    private boolean disabled;
    private boolean finished;
    private boolean stowItSpecial;

    public HoldArmCommand(Arm armSubSystem) {
        super(armSubSystem, false);
    }

    @Override
    public void initialize() {
        super.initialize();
        disabled = false;
        finished = false;
        stowItSpecial = false;
    }

    @Override
    public void execute() {
        if (!disabled) {
            if (stowItSpecial) {
                arm.nudgeDown(0.2);
            } else {
                super.execute();
            }
        }
    }

    public void enable() {
        disabled = false;
        desiredCanCoderPosition = arm.getCANCoderPosition();
    }

    public void disable() {
        disabled = true;
    }

    public void setStowItSpecial(boolean stowItOrNot) {
        stowItSpecial = stowItOrNot;
    }

    public void enableStowItSpecial() {
        setStowItSpecial(true);
    }

    public void disableStowItSpecial() {
        setStowItSpecial(false);
    }

    @Override
    public void end(boolean interrupted){
        // System.out.println("Hold Arm Command exited");
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    public void killIt() {
        finished = true;
    }
}
