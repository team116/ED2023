package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class GyroBalancing extends CommandBase{
    private Swerve swerve;
    private int isBalancedCount = 0;

    public GyroBalancing(Swerve swerveSubsystem){
        swerve = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double pitch = swerve.getPitch();
        if (pitch == 0.0){
            isBalancedCount++;
        }else if (pitch > 3.0){
            isBalancedCount = 0;
            swerve.setSpeedPercent(0.1);
        }else {
            isBalancedCount = 0;
            swerve.setSpeedPercent(-0.1);
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }
}
