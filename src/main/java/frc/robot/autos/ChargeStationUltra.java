package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Direction;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Speed;
import frc.robot.commands.PoleAlignmentCommand;
import frc.robot.autos.primitives.DriveDistanceAtAngle;
import frc.robot.autos.primitives.GrabElement;
import frc.robot.autos.primitives.GrabberExpelCommand;
import frc.robot.autos.primitives.HoldArmCommand;
import frc.robot.autos.primitives.InclineToLevel;
import frc.robot.autos.primitives.MoveArmCommand;
import frc.robot.autos.primitives.RotateInPlaceByEncoders;
import frc.robot.autos.primitives.RotationDirection;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class ChargeStationUltra extends SequentialCommandGroup {
    public ChargeStationUltra(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight,
                        Direction firstStrafeDirection, double strafeDistanceInches,
                        double driveBackDistanceInches, double driveToCenterOfChargeStationInches,
                        RotationDirection firstRotationDirection, double rotationAngleDegrees) {
        BaseHighGoalCone baseHighGoalCone = new BaseHighGoalCone(swerveSubsystem, armSubsystem, grabberSubsystem, limelight);
        HoldArmCommand holdArmCommand = new HoldArmCommand(armSubsystem);

        DriveDistanceAtAngle strafeToLineUpToElement = new DriveDistanceAtAngle(swerveSubsystem, strafeDistanceInches, firstStrafeDirection);

        DriveDistanceAtAngle moveBackwardsFast = new DriveDistanceAtAngle(swerveSubsystem, driveBackDistanceInches, Direction.REVERSE, Speed.FAST);

        RotateInPlaceByEncoders turnAroundToGetElement = new RotateInPlaceByEncoders(swerveSubsystem, rotationAngleDegrees, firstRotationDirection);

        GrabElement grabElement = new GrabElement(armSubsystem, grabberSubsystem, holdArmCommand);

        // FIXME: Might need to make this speed normal, but not sure about time...
        DriveDistanceAtAngle moveBackwardsOntoChargeStation = new DriveDistanceAtAngle(swerveSubsystem, driveToCenterOfChargeStationInches, Direction.REVERSE, Speed.FAST);

        ParallelDeadlineGroup moveBackwardsUntilLevel = new ParallelDeadlineGroup(
            new InclineToLevel(swerveSubsystem.getGyro(), 2.5),
            moveBackwardsOntoChargeStation);

        DriveDistanceAtAngle turnWheels = new DriveDistanceAtAngle(swerveSubsystem, 0.0, Direction.LEFT);

        // REVISIT: Potential Balancing hack by raising arm at the end

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            baseHighGoalCone,
            strafeToLineUpToElement,
            moveBackwardsFast,
            turnAroundToGetElement,
            grabElement,
            moveBackwardsUntilLevel,
            turnWheels,
            new InstantCommand(() -> holdArmCommand.killIt()));

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(holdArmAndOthers);
    }
}
