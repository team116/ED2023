package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Direction;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Speed;
import frc.robot.autos.primitives.DriveDistanceAtAngle;
import frc.robot.autos.primitives.HoldArmCommand;
import frc.robot.autos.primitives.InclineToLevel;
import frc.robot.autos.primitives.MoveArmCommand;
import frc.robot.autos.primitives.RotateInPlaceByGyroInDegrees;
import frc.robot.autos.primitives.RotationDirection;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.Position;

public class ChargeStationBalanceByGyroFast extends SequentialCommandGroup {
    public ChargeStationBalanceByGyroFast(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight) {
        BaseHighGoalConeFast baseHighGoalCone = new BaseHighGoalConeFast(swerveSubsystem, armSubsystem, grabberSubsystem, limelight);
        HoldArmCommand holdArmCommand = new HoldArmCommand(armSubsystem);

        DriveDistanceAtAngle moveBackwardsOntoChargeStationSlow = new DriveDistanceAtAngle(swerveSubsystem, 78.0, Direction.REVERSE, Speed.SLOW);

        ParallelDeadlineGroup moveBackwardsUntilLevel = new ParallelDeadlineGroup(
            new InclineToLevel(swerveSubsystem.getGyro(), 4.0),
            //moveBackwardsOntoChargeStation);
            moveBackwardsOntoChargeStationSlow);

        DriveDistanceAtAngle comeBackForwardOnceGoingLevel = new DriveDistanceAtAngle(swerveSubsystem, 17.0, Direction.FORWARD);
        DriveDistanceAtAngle turnWheels = new DriveDistanceAtAngle(swerveSubsystem, 0.0, Direction.LEFT);

        // REVISIT: Potential Balancing hack by raising arm at the end

        MoveArmCommand stowIt = new MoveArmCommand(armSubsystem, Position.STOWED, 0.5, holdArmCommand);

        DriveDistanceAtAngle moveBackwardsFast = new DriveDistanceAtAngle(swerveSubsystem, 140.0, Direction.REVERSE, Speed.FAST);

        RotateInPlaceByGyroInDegrees turnAroundToGetElement = new RotateInPlaceByGyroInDegrees(swerveSubsystem, 180.0, RotationDirection.CLOCKWISE);

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            stowIt,
            new InstantCommand(() -> holdArmCommand.enableStowItSpecial()),
            moveBackwardsFast,
            turnAroundToGetElement,
            moveBackwardsUntilLevel,
            comeBackForwardOnceGoingLevel,
            turnWheels,
            new InstantCommand(() -> holdArmCommand.killIt()));

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(baseHighGoalCone, holdArmAndOthers);  // NOTE: baseHighGoalCone has own holdArm, so do NOT run in parallel with this one
    }
}
