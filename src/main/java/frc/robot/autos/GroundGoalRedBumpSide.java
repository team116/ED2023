package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;

import static frc.robot.subsystems.Arm.Position.*;
import static frc.robot.autos.DriveDistanceAtAngle.Direction.*;

public class GroundGoalRedBumpSide extends SequentialCommandGroup {

    public GroundGoalRedBumpSide(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem) {
        GrabberIntakeCommand grabConeFromFloorCommand = new GrabberIntakeCommand(grabberSubsystem);

        DriveDistanceAtAngle moveTinyBackwardsAtStart = new DriveDistanceAtAngle(swerveSubsystem, 14.0, REVERSE);

        // REVISIT: Instead of CONE_HIGH_GOAL, might have a special lift strong
        ParallelCommandGroup liftConeFromFloor = new ParallelCommandGroup(
            new MoveArmCommand(armSubsystem, LOW_GOAL),
            moveTinyBackwardsAtStart);

        MoveArmCommand liftArmToScoringPosition = new MoveArmCommand(armSubsystem, LOW_GOAL);

        DriveDistanceAtAngle moveForward = new DriveDistanceAtAngle(swerveSubsystem, 0.0, FORWARD);

        MoveArmCommand stowArm = new MoveArmCommand(armSubsystem, STOWED);

        GrabberExpelCommand scoreCone = new GrabberExpelCommand(grabberSubsystem);

        DriveDistanceAtAngle strafeLeft = new DriveDistanceAtAngle(swerveSubsystem, 4.0, LEFT);

        DriveDistanceAtAngle moveBackwards = new DriveDistanceAtAngle(swerveSubsystem, 150.0, REVERSE);

        addCommands(
            grabConeFromFloorCommand,
            liftConeFromFloor,
            liftArmToScoringPosition,
            moveForward,
            scoreCone,
            stowArm,
            strafeLeft,
            moveBackwards);
    }

}
