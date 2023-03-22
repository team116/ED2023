package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class GyroBalancing extends CommandBase{
    private Swerve swerve;

    public GyroBalancing(Swerve swerveSubsystem){
        swerve = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerve.resetRelativeEncoders();
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
