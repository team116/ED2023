package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSpecificConstants;
import frc.robot.subsystems.Swerve;


public class RotateInPlaceByEncoders extends SequentialCommandGroup {

    private static final double FRONT_TO_BACK_AXLE_TO_AXLE_INCHES = RobotSpecificConstants.getFrontToBackAxleToAxleInches();
    private static final double SIDE_TO_SIDE_TREAD_CENTER_TO_TREAD_CENTER_INCHES = RobotSpecificConstants.getSideToSideTreadCenterToTreadCenterInches();
    private static final double ROTATION_CIRCUMFERENCE_INCHES = 
        (Math.sqrt((FRONT_TO_BACK_AXLE_TO_AXLE_INCHES * FRONT_TO_BACK_AXLE_TO_AXLE_INCHES) + 
                (SIDE_TO_SIDE_TREAD_CENTER_TO_TREAD_CENTER_INCHES * SIDE_TO_SIDE_TREAD_CENTER_TO_TREAD_CENTER_INCHES)) * Math.PI);

    /**
     * 
     * @param swerveSubsystem
     * @param angle positive angle to turn "right?", negative to turn "left?"
     * @param direction
     */
    public RotateInPlaceByEncoders(Swerve swerveSubsystem, double angle) {
        TurnWheelsForRotation turnWheelsForRotation = new TurnWheelsForRotation(swerveSubsystem);
        DriveDistance driveDistance = new DriveDistance(swerveSubsystem,  ((angle / 360.0) * ROTATION_CIRCUMFERENCE_INCHES));

        addCommands(turnWheelsForRotation, driveDistance);
    }
}
