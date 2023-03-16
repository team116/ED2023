package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class DriveRightInInches extends CommandBase{
    private SwerveModulePosition [] listOfModulePositions = new SwerveModulePosition[4];
    private Swerve swerve;
    private int pidSlot = 0;
    private Rotation2d angle = Rotation2d.fromDegrees(90.0d);
    private double distance;
    private Timer timer;

    public DriveRightInInches(Swerve swerveSubsystem, double distanceInches, int pidSlot){
        swerve = swerveSubsystem;
        distanceInches = distance / 39.37;
        this.pidSlot = pidSlot;
        swerve.setPID(10, 0, 0, pidSlot);
        swerve.setMinMax(-0.2, 0.2, pidSlot);
        swerve.burnFlash();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerve.resetRelativeEncoders();
        timer = new Timer();
        timer.start();
    }
    @Override
    public void execute() {
        for (int i = 0; i < listOfModulePositions.length; i++){
            listOfModulePositions[i] = new SwerveModulePosition(distance, angle);
        }
        swerve.setModulePositions(listOfModulePositions, pidSlot);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
