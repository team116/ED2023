package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;

import static frc.robot.subsystems.Arm.Position.*;
import static frc.robot.autos.DriveDistanceAtAngle.Direction.*;

public class HighGoalCone extends SequentialCommandGroup {

    public HighGoalCone(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem) {
        GrabberIntakeCommand grabConeFromFloorCommand = new GrabberIntakeCommand(grabberSubsystem);

        DriveDistanceAtAngle moveTinyBackwardsAtStart = new DriveDistanceAtAngle(swerveSubsystem, 24.0, REVERSE);

        // REVISIT: Instead of CONE_HIGH_GOAL, might have a special lift strong
        ParallelCommandGroup liftConeFromFloor = new ParallelCommandGroup(
            new MoveArmCommand(armSubsystem, LOW_GOAL),
            moveTinyBackwardsAtStart);

        MoveArmCommand liftArmToScoringPosition = new MoveArmCommand(armSubsystem, CONE_HIGH_GOAL);

        DriveDistanceAtAngle moveForward = new DriveDistanceAtAngle(swerveSubsystem, 24.0, FORWARD);

        MoveArmCommand lowerArm = new MoveArmCommand(armSubsystem, CUBE_HIGH_GOAL);

        ParallelCommandGroup driveBackAndScoreCone = new ParallelCommandGroup(new GrabberExpelCommand(grabberSubsystem), 
        new DriveDistanceAtAngle(swerveSubsystem, 28.0, REVERSE));

        MoveArmCommand stowArm = new MoveArmCommand(armSubsystem, STOWED);

        // FIXME: Put back to 150.0 inches... afterwards
        DriveDistanceAtAngle moveBackwards = new DriveDistanceAtAngle(swerveSubsystem, 0.0, REVERSE);

        addCommands(
            grabConeFromFloorCommand,
            liftConeFromFloor,
            liftArmToScoringPosition,
            moveForward,
            lowerArm,
            driveBackAndScoreCone,
            stowArm,
            moveBackwards);
    }
}
