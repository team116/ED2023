package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;

public class GrabElement extends SequentialCommandGroup {

    private static final double MAX_WAIT_SECONDS_FOR_ARM_TO_MOVE = 1.5;
    private final HoldArmCommand holdArmCommand;

    public GrabElement(Arm armSubSystem, Grabber grabberSubsystem, HoldArmCommand holdArmCommand) {
        this.holdArmCommand = holdArmCommand;

        ParallelCommandGroup grabAndIntakeCone = new ParallelCommandGroup(
            new GrabberIntakeCommand(grabberSubsystem),
            new MoveArmCommand(armSubSystem, Arm.Position.LOW_GOAL, MAX_WAIT_SECONDS_FOR_ARM_TO_MOVE, this.holdArmCommand)
        );

        addCommands(grabAndIntakeCone);
    }
}
