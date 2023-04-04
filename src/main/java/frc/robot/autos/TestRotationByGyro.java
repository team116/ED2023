package frc.robot.autos;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.RotateInPlaceByGyroInDegrees;
import frc.robot.subsystems.Swerve;

public class TestRotationByGyro extends SequentialCommandGroup {
    public TestRotationByGyro(Swerve swerveSubsystem, Pigeon2 gyro){
        RotateInPlaceByGyroInDegrees rotate180 = new RotateInPlaceByGyroInDegrees(swerveSubsystem, gyro, 180.0, 0.3);
        
        addCommands(rotate180);
    }
}
