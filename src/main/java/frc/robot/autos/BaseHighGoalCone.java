package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PoleAlignmentCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.subsystems.Arm.Position.*;
import static frc.robot.autos.DriveDistanceAtAngle.Direction.*;

public abstract class BaseHighGoalCone extends SequentialCommandGroup {

    public BaseHighGoalCone(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight) {
        HoldArmCommand holdArmCommand = new HoldArmCommand(armSubsystem);

        DriveDistanceAtAngle moveTinyBackwardsAtStart = new DriveDistanceAtAngle(swerveSubsystem, 24.0, REVERSE);

        SequentialCommandGroup delayedArmCommand = new SequentialCommandGroup(
            new DurationCommand(0.25),
            new MoveArmCommand(armSubsystem, CONE_HIGH_GOAL, 4.0, holdArmCommand));

        ParallelCommandGroup liftConeToHighScoringPositionWhileMovingBack = new ParallelCommandGroup(
            delayedArmCommand,
            moveTinyBackwardsAtStart);

        //MoveArmCommand liftArmToScoringPosition = new MoveArmCommand(armSubsystem, CONE_HIGH_GOAL, 4.0, holdArmCommand);

        DriveDistanceAtAngle moveForward = new DriveDistanceAtAngle(swerveSubsystem, 24.0, FORWARD);

        MoveArmCommand lowerArm = new MoveArmCommand(armSubsystem, CUBE_HIGH_GOAL, 0.25, holdArmCommand);

        SequentialCommandGroup delayedStowArmCommand = new SequentialCommandGroup(
            new DurationCommand(1.0),
            new MoveArmCommand(armSubsystem, STOWED, 1.25, holdArmCommand)
        );

        ParallelCommandGroup scoreConeWhileDrivingBackAndStowingArm = new ParallelCommandGroup(
            new GrabberExpelCommand(grabberSubsystem),
            new DriveDistanceAtAngle(swerveSubsystem, 28.0, REVERSE),
            delayedStowArmCommand);

        PoleAlignmentCommand autoAlign = new PoleAlignmentCommand(swerveSubsystem, limelight);

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            liftConeToHighScoringPositionWhileMovingBack,
            moveForward,
            autoAlign,
            lowerArm,
            scoreConeWhileDrivingBackAndStowingArm,
            new InstantCommand(() -> holdArmCommand.killIt()));

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(holdArmAndOthers);
    }

}
