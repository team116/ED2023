package frc.robot.autos;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Swerve;

public class DriveDistance extends DurationCommand {
    private static final int PID_SLOT = 1;
    private static final double WORST_CASE_INCHES_PER_SECOND = 8.0;
    private static final double METERS_AWAY_FROM_DESIRED_THRESHOLD = 0.05;

    private Swerve swerve;
    private double distance;

    /**
     * Positive distance to move forward, and negative distance to move backward.
     * 
     * @param swerveSubsystem the swerve subsystem
     * @param distanceInches positive and negative distance in inches
     * @param angleDegrees the direction to move
     */
    public DriveDistance(Swerve swerveSubsystem, double distanceInches) {
        super(deriveMaxTimeoutFromDistance(distanceInches));
        this.swerve = swerveSubsystem;
        this.distance = Units.inchesToMeters(distanceInches);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        swerve.resetDriveEncoders();
        swerve.runToPosition(distance, PID_SLOT);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop(); // NOTE: Ensure motors are no longer moving...
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
          // Only exit after all are at positions, or timer hits timeout
        return (allAtFinalPositions() || super.isFinished());
    }

    private boolean allAtFinalPositions() {
        for (SwerveModulePosition currentPosition : swerve.getPositions()) {
            if (Math.abs(currentPosition.distanceMeters - distance) > METERS_AWAY_FROM_DESIRED_THRESHOLD) {
                return false;
            }
        }

        return true;
    }

    private static double deriveMaxTimeoutFromDistance(double distanceInches) {
        // NOTE: All times should be non-negative
        return Math.abs(distanceInches / WORST_CASE_INCHES_PER_SECOND);
    }
}
