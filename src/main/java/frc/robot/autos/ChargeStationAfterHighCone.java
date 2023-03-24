package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.autos.DriveDistanceAtAngle.Direction.*;

public class ChargeStationAfterHighCone extends BaseHighGoalCone {

    public ChargeStationAfterHighCone(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight) {
        super(swerveSubsystem, armSubsystem, grabberSubsystem, limelight);

        HoldArmCommand holdArmCommand = new HoldArmCommand(armSubsystem);

        // Start of after high cone score
        DriveDistanceAtAngle moveBackwardsOverChargeStation = new DriveDistanceAtAngle(swerveSubsystem, 150.0, REVERSE);

        DriveDistanceAtAngle moveForwardsOnToChargeStation = new DriveDistanceAtAngle(swerveSubsystem, 40.0, FORWARD);

        DriveDistanceAtAngle turnWheels = new DriveDistanceAtAngle(swerveSubsystem, 0.0, LEFT);

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            moveBackwardsOverChargeStation,
            moveForwardsOnToChargeStation,
            turnWheels);

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(holdArmAndOthers);
    }

}
