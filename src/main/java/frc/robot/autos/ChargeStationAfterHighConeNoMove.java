package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

public class ChargeStationAfterHighConeNoMove extends BaseHighGoalCone {

    public ChargeStationAfterHighConeNoMove(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight) {
        super(swerveSubsystem, armSubsystem, grabberSubsystem, limelight);

        HoldArmCommand holdArmCommand = new HoldArmCommand(armSubsystem);

        // Start of after high cone
        MoveArmCommand stowArm = new MoveArmCommand(armSubsystem, STOWED, 0.25, holdArmCommand);
        DriveDistanceAtAngle moveForwardOutOfWay = new DriveDistanceAtAngle(swerveSubsystem, 24.0, FORWARD, Speed.NORMAL);

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            stowArm,
            moveForwardOutOfWay,
            new InstantCommand(() -> holdArmCommand.killIt()));

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(holdArmAndOthers);
    }

}
