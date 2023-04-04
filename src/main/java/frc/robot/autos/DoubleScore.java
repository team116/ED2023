package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Direction;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Speed;
import frc.robot.commands.PoleAlignmentCommand;
import frc.robot.autos.primitives.DriveDistanceAtAngle;
import frc.robot.autos.primitives.GrabElement;
import frc.robot.autos.primitives.GrabberExpelCommand;
import frc.robot.autos.primitives.HoldArmCommand;
import frc.robot.autos.primitives.MoveArmCommand;
import frc.robot.autos.primitives.RotateInPlaceByEncoders;
import frc.robot.autos.primitives.RotationDirection;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class DoubleScore extends SequentialCommandGroup {
    public DoubleScore(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight,
                        Direction firstStrafeDirection, double strafeDistanceInches,
                        double driveBackDistanceInches, double driveForwardDistanceInches,
                        RotationDirection firstRotationDirection, double rotationAngleDegrees) {
        BaseHighGoalCone baseHighGoalCone = new BaseHighGoalCone(swerveSubsystem, armSubsystem, grabberSubsystem, limelight);
        HoldArmCommand holdArmCommand = new HoldArmCommand(armSubsystem);

        DriveDistanceAtAngle strafeToLineUpToElement = new DriveDistanceAtAngle(swerveSubsystem, strafeDistanceInches, firstStrafeDirection);

        DriveDistanceAtAngle moveBackwardsFast = new DriveDistanceAtAngle(swerveSubsystem, driveBackDistanceInches, Direction.REVERSE, Speed.FAST);

        RotateInPlaceByEncoders turnAroundToGetElement = new RotateInPlaceByEncoders(swerveSubsystem, rotationAngleDegrees, firstRotationDirection);

        GrabElement grabElement = new GrabElement(armSubsystem, grabberSubsystem, holdArmCommand);

        RotateInPlaceByEncoders turnBackAround = new RotateInPlaceByEncoders(swerveSubsystem, rotationAngleDegrees, firstRotationDirection.getInverse());

        DriveDistanceAtAngle moveForwardsFast = new DriveDistanceAtAngle(swerveSubsystem, driveForwardDistanceInches, Direction.FORWARD, Speed.FAST);
        MoveArmCommand liftToMidConeScore = new MoveArmCommand(armSubsystem, Arm.Position.CONE_HIGH_GOAL, 3.0, holdArmCommand);
        ParallelCommandGroup liftToScoreAndMoveForwardsFast = new ParallelCommandGroup(moveForwardsFast, liftToMidConeScore);

        DriveDistanceAtAngle strafeToLineUpWithPole = new DriveDistanceAtAngle(swerveSubsystem, strafeDistanceInches, firstStrafeDirection.getInverse());

        PoleAlignmentCommand autoAlign = new PoleAlignmentCommand(swerveSubsystem, limelight);

        MoveArmCommand lowerArm = new MoveArmCommand(armSubsystem, Arm.Position.AUTO_CONE_MID_GOAL_SCORE, 0.25, holdArmCommand);

        ParallelCommandGroup scoreConeWhileDrivingBack = new ParallelCommandGroup(
            new GrabberExpelCommand(grabberSubsystem),
            new DriveDistanceAtAngle(swerveSubsystem, 28.0, Direction.REVERSE)
        );

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            baseHighGoalCone,
            strafeToLineUpToElement,
            moveBackwardsFast,
            turnAroundToGetElement,
            grabElement,
            turnBackAround,
            liftToScoreAndMoveForwardsFast,
            strafeToLineUpWithPole,
            autoAlign,
            lowerArm,
            scoreConeWhileDrivingBack,
            new InstantCommand(() -> holdArmCommand.killIt()));

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(holdArmAndOthers);
    }
}
