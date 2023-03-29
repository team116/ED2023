package frc.robot.autos;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.Swerve;

public class RotateInPlaceInDegrees extends DurationCommand{
    Swerve swerve;
    Pigeon2 gyro;
    double deisredAngleDegrees;
    double percentPower;
    int pidSlot;
    int atAngleCount = 0;


    public RotateInPlaceInDegrees(Swerve swerveSubsystem, Pigeon2 gyro, double deisredAngleDegrees, double percentPower){
        super(1.0);
        this.gyro = gyro;
        this.deisredAngleDegrees = deisredAngleDegrees;
        this.percentPower = percentPower;
        swerve = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();    
        SwerveModulePosition [] modAngles = new SwerveModulePosition[]{
                                                                new SwerveModulePosition(0, Rotation2d.fromDegrees(45)),
                                                                new SwerveModulePosition(0, Rotation2d.fromDegrees(135)),
                                                                new SwerveModulePosition(0, Rotation2d.fromDegrees(225)),
                                                                new  SwerveModulePosition(0, Rotation2d.fromDegrees(315))};
        for (SwerveModulePosition angle : modAngles){
            swerve.turnWheelsToToAngle(angle.angle);
        }
        atAngleCount = 0;
    }

    @Override
    public void execute() {
        if (gyro.getYaw() < deisredAngleDegrees || gyro.getYaw() > deisredAngleDegrees){
            swerve.setSpeedPercent(percentPower);
            atAngleCount = 0;
        }else {
            swerve.setSpeedPercent(0);
            atAngleCount++;
        }
        
    }

    @Override
    public boolean isFinished() {
        return (super.isFinished() || atAngleCount > 3);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
