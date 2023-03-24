package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PoleAlignmentCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.autos.DriveDistanceAtAngle.Direction.*;

public class HighGoalCone extends BaseHighGoalCone {

    public HighGoalCone(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight) {
        super(swerveSubsystem, armSubsystem, grabberSubsystem, limelight);
        HoldArmCommand holdArmCommand = new HoldArmCommand(armSubsystem);

        // FIXME: Put back to 150.0 inches... afterwards
        DriveDistanceAtAngle moveBackwards = new DriveDistanceAtAngle(swerveSubsystem, 6.0, REVERSE);

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            moveBackwards);

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(holdArmAndOthers);
    }
}
