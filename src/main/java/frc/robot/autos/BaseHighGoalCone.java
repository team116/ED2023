package frc.robot.autos;

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

        // REVISIT: Instead of CONE_HIGH_GOAL, might have a special lift strong
        ParallelCommandGroup liftConeFromFloor = new ParallelCommandGroup(
            new MoveArmCommand(armSubsystem, LOW_GOAL, 2.0, holdArmCommand),
            moveTinyBackwardsAtStart);

        MoveArmCommand liftArmToScoringPosition = new MoveArmCommand(armSubsystem, CONE_HIGH_GOAL, 4.0, holdArmCommand);

        DriveDistanceAtAngle moveForward = new DriveDistanceAtAngle(swerveSubsystem, 24.0, FORWARD);

        MoveArmCommand lowerArm = new MoveArmCommand(armSubsystem, CUBE_HIGH_GOAL, 0.25, holdArmCommand);

        ParallelCommandGroup driveBackAndScoreCone = new ParallelCommandGroup(new GrabberExpelCommand(grabberSubsystem), 
        new DriveDistanceAtAngle(swerveSubsystem, 28.0, REVERSE));

        MoveArmCommand stowArm = new MoveArmCommand(armSubsystem, STOWED, 1.5, holdArmCommand);

        PoleAlignmentCommand autoAlign = new PoleAlignmentCommand(swerveSubsystem, limelight);

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            liftConeFromFloor,
            liftArmToScoringPosition,
            moveForward,
            autoAlign,
            lowerArm,
            driveBackAndScoreCone,
            stowArm);

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        // FIXME:
        //addCommands(holdArmAndOthers);
    }
}
