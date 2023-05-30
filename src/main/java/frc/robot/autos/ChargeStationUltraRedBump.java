package frc.robot.autos;

import frc.robot.autos.primitives.RotationDirection;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Direction;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class ChargeStationUltraRedBump extends ChargeStationUltra {
    public ChargeStationUltraRedBump(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight) {
        super(swerveSubsystem,
            armSubsystem,
            grabberSubsystem,
            limelight,
            Direction.RIGHT,
            2.0,  // 1.0 - 2.0
            150.0,
            40.0,
            RotationDirection.COUNTER_CLOCKWISE,
            180.0);
    }
}
