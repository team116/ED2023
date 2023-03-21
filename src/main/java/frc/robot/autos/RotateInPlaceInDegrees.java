package frc.robot.autos;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class RotateInPlaceInDegrees extends CommandBase{
    Swerve swerve;
    Pigeon2 gyro;
    double deisredAngleDegrees;
    double percentPower;
    int pidSlot;

    public RotateInPlaceInDegrees(Swerve swerveSubsystem, Pigeon2 gyro, double deisredAngleDegrees, double percentPower, int pidSlot){
        this.gyro = gyro;
        this.deisredAngleDegrees = deisredAngleDegrees;
        this.percentPower = percentPower;
        swerve = swerveSubsystem;
        this.pidSlot = pidSlot;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SwerveModulePosition [] modAngles = new SwerveModulePosition[]{
                                                                new SwerveModulePosition(0, Rotation2d.fromDegrees(45)),
                                                                new SwerveModulePosition(0, Rotation2d.fromDegrees(135)),
                                                                new SwerveModulePosition(0, Rotation2d.fromDegrees(225)),
                                                                new  SwerveModulePosition(0, Rotation2d.fromDegrees(deisredAngleDegrees))};
    }
}
