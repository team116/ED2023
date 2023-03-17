package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class DriveDistanceAtAngle extends CommandBase {
    private Swerve swerve;
    private double distance;
    private int pidSlot = 1;
    private Rotation2d angle;
    private double startTime;

    public enum Direction {
        FORWARD(0.0d, 1.0),
        REVERSE(0.0d, -1.0),
        LEFT(90.0d, 1.0),
        RIGHT(90.0d, -1.0),
        DIAGONAL_FORWARD_LEFT(45.0d, 1.0),
        DIAGONAL_FORWARD_RIGHT(-45.0d, 1.0),
        DIAGONAL_BACKWARD_LEFT(-45.0d, -1.0),
        DIAGONAL_BACKWARD_RIGHT(45.0d, -1.0);

        public final double angleDegrees;
        public final double directionMultiplier;

        private Direction(double angleDegrees, double directionMultiplier) {
            this.angleDegrees = angleDegrees;
            this.directionMultiplier = directionMultiplier;
        }
    }
    
    /**
     * The constructor to utilize to move the robot in the indicated direction for the distance specified in inches.
     * 
     * @param swerveSubsystem the swerve subsystem
     * @param distanceInches a positive value for the distance to drive
     * @param direction the directioon to move
     */
    public DriveDistanceAtAngle(Swerve swerveSubsystem, double distanceInches, Direction direction) {
        this(swerveSubsystem, distanceInches * direction.directionMultiplier, direction.angleDegrees);
    }

    /**
     * Constructor that really shouldn't need to be used, as Direction based constructor is preferred.
     * 
     * @param swerveSubsystem the swerve subsystem
     * @param distanceInches positive and negative distance in inches
     * @param angleDegrees the direction to move
     */
    public DriveDistanceAtAngle(Swerve swerveSubsystem, double distanceInches, double angleDegrees) {
        this.swerve = swerveSubsystem;
        this.distance = Units.inchesToMeters(distanceInches);
        this.angle = Rotation2d.fromDegrees(angleDegrees);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerve.resetRelativeEncoders();
        startTime = Timer.getFPGATimestamp();

        SwerveModulePosition [] listOfModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < listOfModulePositions.length; i++){
            listOfModulePositions[i] = new SwerveModulePosition(distance, angle);
        }
        swerve.setModulePositions(listOfModulePositions, pidSlot);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop(); // NOTE: Ensure motors are no longer moving...
    }

    @Override
    public boolean isFinished() {
          // Only exit after all are at positions, or timer hits timeout
        return (allAtFinalPositions() || (Timer.getFPGATimestamp() - startTime > 1.5d)); // NOTE: Duration should be estimate based upon distance
    }

    private boolean allAtFinalPositions() {
        for (SwerveModulePosition currentPosition : swerve.getPositions()) {
            if (Math.abs(currentPosition.distanceMeters - distance) > 0.05) {
                return false;
            }
        }

        return true;
    } 
}
