package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class DriveDistanceAtAngle extends SequentialCommandGroup {

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
        TurnWheelsToAngle turnWheelsToAngle = new TurnWheelsToAngle(swerveSubsystem, angleDegrees);
        DriveDistance driveDistance = new DriveDistance(swerveSubsystem, distanceInches);

        addCommands(
            turnWheelsToAngle,
            driveDistance
        );
    }

}
