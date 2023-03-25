package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.DriveDistanceAtAngle.Speed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.subsystems.Arm.Position.*;
import static frc.robot.autos.DriveDistanceAtAngle.Direction.*;

public class ChargeStationAfterHighConeSimple extends BaseHighGoalCone {

    public ChargeStationAfterHighConeSimple(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight) {
        super(swerveSubsystem, armSubsystem, grabberSubsystem, limelight);

        HoldArmCommand holdArmCommand = new HoldArmCommand(armSubsystem);

        // Start of after high cone
        MoveArmCommand stowArm = new MoveArmCommand(armSubsystem, STOWED, 0.25, holdArmCommand);
        DriveDistanceAtAngle moveBackwardsOntoChargeStation = new DriveDistanceAtAngle(swerveSubsystem, 77.0, REVERSE, Speed.NORMAL);
        DriveDistanceAtAngle turnWheels = new DriveDistanceAtAngle(swerveSubsystem, 0.0, LEFT);

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            stowArm,
            moveBackwardsOntoChargeStation,
            turnWheels,
            new InstantCommand(() -> holdArmCommand.killIt()));

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(holdArmAndOthers);
    }

}
