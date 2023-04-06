package frc.robot.autos;

import frc.robot.autos.primitives.RotationDirection;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Direction;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class DoubleScoreRedBump extends DoubleScore {
    public DoubleScoreRedBump(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight) {
        super(swerveSubsystem,
            armSubsystem,
            grabberSubsystem,
            limelight,
            Direction.LEFT,
            12.0,
            150.0,
            178.0,
            RotationDirection.COUNTER_CLOCKWISE,
            180.0);
    }
}
