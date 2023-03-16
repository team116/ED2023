package frc.robot.autos;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class ChargingStation extends CommandBase{
    private Swerve swerve;
    private Pigeon2 gyro;
    private int pidSlot = 1;

    public ChargingStation(Swerve swerveSubsystem, Pigeon2 gyro){
        this.gyro = gyro;

        swerve = swerveSubsystem;
        swerve.setPID(10, 0, 0, pidSlot);
        swerve.setMinMax(-0.2, 0.2, pidSlot);
        swerve.burnFlash();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
