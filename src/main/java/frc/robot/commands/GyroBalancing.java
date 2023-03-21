package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class GyroBalancing extends CommandBase{
    private Pigeon2 gyro;
    private Swerve swerve;

    public GyroBalancing(Pigeon2 gyro, Swerve swerveSubsystem){
        this.gyro = gyro;
        swerve = swerveSubsystem;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }
}
