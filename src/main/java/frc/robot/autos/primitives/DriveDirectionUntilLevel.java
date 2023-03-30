package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Direction;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Speed;
import frc.robot.subsystems.Swerve;

public class DriveDirectionUntilLevel extends ParallelDeadlineGroup {

    private static class GyroLevelDeadline extends CommandBase {
        private int stabilizedCount;
        private double startTime;
        private final Swerve swerveDriveSubsystem;

        public GyroLevelDeadline(Swerve swerveDriveSubsystem) {
            this.swerveDriveSubsystem = swerveDriveSubsystem;
        }

        @Override
        public void initialize() {
            startTime = Timer.getFPGATimestamp();
            stabilizedCount = 0;
        }
    
        // What???? The gyro is between 1.5 to 3.0 driving on carpet ???
        @Override
        public void execute() {
            // System.out.println("pitch: " + swerveDriveSubsystem.getPitch());
            // System.out.println("roll: " + swerveDriveSubsystem.getRoll());
            if ((Math.abs(swerveDriveSubsystem.getPitch()) - 2.25) > 0.75) {
                stabilizedCount = 0;
            } else {
                ++stabilizedCount;
            }
        }
    
        @Override
        public boolean isFinished() {
              // Only exit after 3 non-movements, or timer hits half second
            return (stabilizedCount > 3 || (Timer.getFPGATimestamp() - startTime > 1.5d));
        }
        
    }

    public DriveDirectionUntilLevel(Swerve swerveSubsystem, Direction direction) {
        super(
            new GyroLevelDeadline(swerveSubsystem),
            new DriveDistanceAtAngle(swerveSubsystem, 40.0, direction, Speed.FAST)
        );
    }
}
