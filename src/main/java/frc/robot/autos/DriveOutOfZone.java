package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

import static frc.robot.autos.DriveDistanceAtAngle.Direction.*;

public class DriveOutOfZone extends SequentialCommandGroup {
    private Swerve swerve;

    public DriveOutOfZone(Swerve swerveSubsystem) {
        swerve = swerveSubsystem;

        addCommands(
            new DriveDistanceAtAngle(swerve, 12.0, REVERSE),
            new DriveDistanceAtAngle(swerve, 12.0, FORWARD),
            new DriveDistanceAtAngle(swerve, 12.0, LEFT),
            new DriveDistanceAtAngle(swerve, 12.0, RIGHT),
            new DriveDistanceAtAngle(swerve, 12.0, DIAGONAL_BACKWARD_LEFT),
            new DriveDistanceAtAngle(swerve, 12.0, DIAGONAL_FORWARD_RIGHT),
            new DriveDistanceAtAngle(swerve, 12.0, DIAGONAL_BACKWARD_RIGHT),
            new DriveDistanceAtAngle(swerve, 12.0, DIAGONAL_FORWARD_LEFT));
    }
}
