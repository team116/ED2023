package frc.robot.autos.primitives;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.Swerve;

public class RotateInPlaceByGyroInDegrees extends DurationCommand{
    private Swerve swerve;
    private Pigeon2 gyro;
    private double deisredAngleDegrees;
    private double percentPower;
    private int atAngleCount = 0;
    private static final double DEGREES_THERESHHOLD = 0.25;


    public RotateInPlaceByGyroInDegrees(Swerve swerveSubsystem, Pigeon2 gyro, double deisredAngleDegrees, double percentPower){
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
        double difference = Math.abs(gyro.getYaw() - deisredAngleDegrees);
        if (difference > DEGREES_THERESHHOLD){
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
