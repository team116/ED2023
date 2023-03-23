package frc.robot.autos;

import frc.robot.commands.BaseArmCommand;
import frc.robot.subsystems.Arm;

public class HoldArmCommand extends BaseArmCommand {
    private boolean disabled;

    public HoldArmCommand(Arm armSubSystem) {
        super(armSubSystem, false);
    }

    @Override
    public void initialize() {
        super.initialize();
        disabled = false;
    }

    @Override
    public void execute() {
        if (!disabled) {
            super.execute();
        }
    }

    public void enable() {
        disabled = false;
        desiredCanCoderPosition = arm.getCANCoderPosition();
    }

    public void disable() {
        disabled = true;
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("Hold Arm Command exited");
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
