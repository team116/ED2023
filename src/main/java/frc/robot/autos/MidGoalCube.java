package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DriveDistanceAtAngle;
import frc.robot.autos.primitives.GrabberExpelCommand;
import frc.robot.autos.primitives.GrabberIntakeCommand;
import frc.robot.autos.primitives.MoveArmCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;

import static frc.robot.autos.primitives.DriveDistanceAtAngle.Direction.*;
import static frc.robot.subsystems.Arm.Position.*;

public class MidGoalCube extends SequentialCommandGroup{
    public MidGoalCube(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem) {
        GrabberIntakeCommand grabConeFromFloorCommand = new GrabberIntakeCommand(grabberSubsystem);

        DriveDistanceAtAngle moveTinyBackwardsAtStart = new DriveDistanceAtAngle(swerveSubsystem, 12.0, REVERSE);

        // REVISIT: Instead of CONE_HIGH_GOAL, might have a special lift strong
        ParallelCommandGroup liftConeFromFloor = new ParallelCommandGroup(
            new MoveArmCommand(armSubsystem, LOW_GOAL),
            moveTinyBackwardsAtStart);

        MoveArmCommand liftArmToScoringPosition = new MoveArmCommand(armSubsystem, CUBE_MID_GOAL);

        DriveDistanceAtAngle moveForward = new DriveDistanceAtAngle(swerveSubsystem, 18.0, FORWARD);

        GrabberExpelCommand scoreCone = new GrabberExpelCommand(grabberSubsystem);

        DriveDistanceAtAngle moveBackwards = new DriveDistanceAtAngle(swerveSubsystem, 6.0, REVERSE);

        addCommands(
            grabConeFromFloorCommand,
            liftConeFromFloor,
            liftArmToScoringPosition,
            moveForward,
            scoreCone,
            moveBackwards);
    }
}
