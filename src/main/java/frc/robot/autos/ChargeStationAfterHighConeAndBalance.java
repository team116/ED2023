package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DriveDistanceAtAngle;
import frc.robot.autos.primitives.GyroBalancing;
import frc.robot.autos.primitives.HoldArmCommand;
import frc.robot.autos.primitives.MoveArmCommand;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Speed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.autos.primitives.DriveDistanceAtAngle.Direction.*;
import static frc.robot.subsystems.Arm.Position.*;

public class ChargeStationAfterHighConeAndBalance extends BaseHighGoalCone {

    public ChargeStationAfterHighConeAndBalance(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight) {
        super(swerveSubsystem, armSubsystem, grabberSubsystem, limelight);

        HoldArmCommand holdArmCommand = new HoldArmCommand(armSubsystem);

        // Start of after high cone
        MoveArmCommand stowArm = new MoveArmCommand(armSubsystem, STOWED, 0.25, holdArmCommand);

        // NOTE: Ensure we are not completely balanced at this point, but at least on the charge pad at a significant angle
        DriveDistanceAtAngle moveBackwardsOntoChargeStation = new DriveDistanceAtAngle(swerveSubsystem, 74.0, REVERSE, Speed.NORMAL);

        DriveDistanceAtAngle turnWheels = new DriveDistanceAtAngle(swerveSubsystem, 0.0, LEFT);

        GyroBalancing gyroBalancing = new GyroBalancing(swerveSubsystem.getGyro(), swerveSubsystem, 3, REVERSE);

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            stowArm,
            new InstantCommand(() -> holdArmCommand.enableStowItSpecial()),
            moveBackwardsOntoChargeStation,
            gyroBalancing,
            turnWheels,
            new InstantCommand(() -> holdArmCommand.killIt()));

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(holdArmAndOthers);
    }

}
