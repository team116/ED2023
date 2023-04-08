package frc.robot.autos.primitives;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SeeGyro extends CommandBase{
    private Pigeon2 gyro;

    public SeeGyro(Pigeon2 gyro){
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    SmartDashboard.putNumber("Pitch", gyro.getPitch());
    }
}
