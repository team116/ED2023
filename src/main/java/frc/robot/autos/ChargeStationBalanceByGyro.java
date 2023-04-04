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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.Position;

public class ChargeStationBalanceByGyro extends SequentialCommandGroup {
    public ChargeStationBalanceByGyro(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight) {
        BaseHighGoalCone baseHighGoalCone = new BaseHighGoalCone(swerveSubsystem, armSubsystem, grabberSubsystem, limelight);
        HoldArmCommand holdArmCommand = new HoldArmCommand(armSubsystem);

        DriveDistanceAtAngle moveBackwardsOntoChargeStationSlow = new DriveDistanceAtAngle(swerveSubsystem, 78.0, Direction.REVERSE, Speed.SLOW);

        ParallelDeadlineGroup moveBackwardsUntilLevel = new ParallelDeadlineGroup(
            new InclineToLevel(swerveSubsystem.getGyro(), 4.0),
            //moveBackwardsOntoChargeStation);
            moveBackwardsOntoChargeStationSlow);

        DriveDistanceAtAngle comeBackForwardOnceGoingLevel = new DriveDistanceAtAngle(swerveSubsystem, 12.0, Direction.FORWARD);
        DriveDistanceAtAngle turnWheels = new DriveDistanceAtAngle(swerveSubsystem, 0.0, Direction.LEFT);

        // REVISIT: Potential Balancing hack by raising arm at the end

        MoveArmCommand stowIt = new MoveArmCommand(armSubsystem, Position.STOWED, 0.5, holdArmCommand);

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            stowIt,
            moveBackwardsUntilLevel,
            comeBackForwardOnceGoingLevel,
            turnWheels,
            new InstantCommand(() -> holdArmCommand.killIt()));

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(baseHighGoalCone, holdArmAndOthers);  // NOTE: baseHighGoalCone has own holdArm, so do NOT run in parallel with this one
    }
}
