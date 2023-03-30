package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DriveDirectionUntilLevel;
import frc.robot.autos.primitives.DriveDistanceAtAngle;
import frc.robot.autos.primitives.HoldArmCommand;
import frc.robot.autos.primitives.MoveArmCommand;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Speed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.autos.primitives.DriveDistanceAtAngle.Direction.*;
import static frc.robot.subsystems.Arm.Position.*;

public class ChargeStationAfterHighCone extends BaseHighGoalCone {

    public ChargeStationAfterHighCone(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight) {
        super(swerveSubsystem, armSubsystem, grabberSubsystem, limelight);

        HoldArmCommand holdArmCommand = new HoldArmCommand(armSubsystem);

        // Start of after high cone
        DriveDistanceAtAngle moveBackwardsOverChargeStation = new DriveDistanceAtAngle(swerveSubsystem, 150.0, REVERSE, Speed.FAST);

        DriveDistanceAtAngle moveForwardsOnToChargeStation = new DriveDistanceAtAngle(swerveSubsystem, 40.0, FORWARD, Speed.FAST);

        DriveDistanceAtAngle turnWheels = new DriveDistanceAtAngle(swerveSubsystem, 0.0, LEFT);

        MoveArmCommand stowArm = new MoveArmCommand(armSubsystem, STOWED, 0.25, holdArmCommand);
        MoveArmCommand stowArm2 = new MoveArmCommand(armSubsystem, STOWED, 0.25, holdArmCommand);

        DriveDirectionUntilLevel driveToBalanceOnChargeStation = new DriveDirectionUntilLevel(swerveSubsystem, FORWARD);

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            stowArm,
            moveBackwardsOverChargeStation,
            stowArm2,
            moveForwardsOnToChargeStation,
            driveToBalanceOnChargeStation,
            turnWheels,
            new InstantCommand(() -> holdArmCommand.killIt()));

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(holdArmAndOthers);
    }

}
