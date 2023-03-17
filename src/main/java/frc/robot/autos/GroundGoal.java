package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;

import static frc.robot.subsystems.Arm.Position.*;
import static frc.robot.autos.DriveDistanceAtAngle.Direction.*;

public class GroundGoal extends SequentialCommandGroup {

    public GroundGoal(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem) {
        ParallelCommandGroup grabConeFromFloorCommand = new ParallelCommandGroup(
            new MoveArmCommand(armSubsystem, FLOOR_INTAKE),
            new GrabberIntakeCommand(grabberSubsystem));

        MoveArmCommand liftArmToScoringPosition = new MoveArmCommand(armSubsystem, LOW_GOAL);

        DriveDistanceAtAngle moveForward = new DriveDistanceAtAngle(swerveSubsystem, 12.0, FORWARD);

        GrabberExpelCommand scoreCone = new GrabberExpelCommand(grabberSubsystem);

        DriveDistanceAtAngle moveBackwards = new DriveDistanceAtAngle(swerveSubsystem, 12.0, REVERSE);

        addCommands(
            grabConeFromFloorCommand,
            liftArmToScoringPosition,
            moveForward,
            scoreCone,
            moveBackwards);
    }

}
