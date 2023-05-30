package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSpecificConstants;
import frc.robot.subsystems.Swerve;


public class RotateInPlaceByEncoders extends SequentialCommandGroup {

    private static final double FRONT_TO_BACK_AXLE_TO_AXLE_INCHES = RobotSpecificConstants.getFrontToBackAxleToAxleInches();
    private static final double SIDE_TO_SIDE_TREAD_CENTER_TO_TREAD_CENTER_INCHES = RobotSpecificConstants.getSideToSideTreadCenterToTreadCenterInches();
    private static final double COMPUTED_ROTATION_DIAMETER_INCHES = Math.sqrt(
        (FRONT_TO_BACK_AXLE_TO_AXLE_INCHES * FRONT_TO_BACK_AXLE_TO_AXLE_INCHES) + 
        (SIDE_TO_SIDE_TREAD_CENTER_TO_TREAD_CENTER_INCHES * SIDE_TO_SIDE_TREAD_CENTER_TO_TREAD_CENTER_INCHES));

    private static final double EMPERICAL_ROTATION_DIAMETER_INCHES = 36.25; // Measured is 35", but turning radius makes it not perfect
    private static final double ROTATION_CIRCUMFERENCE_INCHES = (EMPERICAL_ROTATION_DIAMETER_INCHES * Math.PI);

    /**
     * 
     * @param swerveSubsystem
     * @param angleDegrees positive angle to turn CLOCKWISE "right", negative to turn COUNTER_CLOCKWISE "left"
     * @param direction
     */
    public RotateInPlaceByEncoders(Swerve swerveSubsystem, double angleDegrees) {
        TurnWheelsForRotation turnWheelsForRotation = new TurnWheelsForRotation(swerveSubsystem);
        DriveDistance driveDistance = new DriveDistance(swerveSubsystem,  -((angleDegrees / 360.0) * ROTATION_CIRCUMFERENCE_INCHES), DriveDistance.SLOW_PID_SLOT);

        addCommands(turnWheelsForRotation, driveDistance);
    }

    public RotateInPlaceByEncoders(Swerve swerveSubsystem, double angleDegrees, RotationDirection rotationDirection) {
        this(swerveSubsystem, angleDegrees * rotationDirection.getDirectionModifier());
    }
}
