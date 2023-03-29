package frc.robot.autos;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.DriveDistanceAtAngle.Speed;
import frc.robot.commands.PoleAlignmentCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.subsystems.Arm.Position.*;
import static frc.robot.autos.DriveDistanceAtAngle.Direction.*;

public class PickUpSecondPieceAfterHighConeAndScoreInGroundGoal extends SequentialCommandGroup{
    public PickUpSecondPieceAfterHighConeAndScoreInGroundGoal(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight, Pigeon2 gyro){
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

        SequentialCommandGroup delayedCarryArmCommand = new SequentialCommandGroup(
            new DurationCommand(1.0),
            new MoveArmCommand(armSubsystem, LOW_GOAL, 1.25, holdArmCommand)
        );

        ParallelCommandGroup scoreConeWhileDrivingBackAndCarryingArm = new ParallelCommandGroup(
            new GrabberExpelCommand(grabberSubsystem),
            new DriveDistanceAtAngle(swerveSubsystem, 220.0, REVERSE, Speed.FAST),
            delayedCarryArmCommand);

        PoleAlignmentCommand autoAlign = new PoleAlignmentCommand(swerveSubsystem, limelight);

        RotateInPlaceInDegrees rotate180 = new RotateInPlaceInDegrees(swerveSubsystem, gyro, 180, 0.1);

        DriveDistanceAtAngle moveLeft = new DriveDistanceAtAngle(swerveSubsystem, 4, LEFT);

        ParallelCommandGroup driveForwardsAndIntakeCone = new ParallelCommandGroup(
            new DriveDistanceAtAngle(swerveSubsystem, 4, FORWARD), 
            new GrabberIntakeCommand(grabberSubsystem));

        DriveDistanceAtAngle moveRight = new DriveDistanceAtAngle(swerveSubsystem, 4, RIGHT);

        RotateInPlaceInDegrees rotateback180 = new RotateInPlaceInDegrees(swerveSubsystem, gyro, 0, 0.1);

        DriveDistanceAtAngle moveBackToGroundGoal = new DriveDistanceAtAngle(swerveSubsystem, 220.0, FORWARD);

        GrabberExpelCommand scoreConeInGround = new GrabberExpelCommand(grabberSubsystem);

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            liftConeToHighScoringPositionWhileMovingBack,
            moveForward,
            autoAlign,
            lowerArm,
            scoreConeWhileDrivingBackAndCarryingArm,
            rotate180,
            moveLeft,
            driveForwardsAndIntakeCone,
            moveRight,
            rotateback180,
            moveBackToGroundGoal,
            scoreConeInGround,
            new InstantCommand(() -> holdArmCommand.killIt()));

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(holdArmAndOthers);
    
    }
}
