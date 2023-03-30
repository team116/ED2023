package frc.robot.autos.primitives;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.Swerve;

public class TurnWheelsForRotation extends DurationCommand {

    private static final double MAX_SECONDS_TO_TURN_WHEELS = 0.5;
    private static final double DEGRESS_AWAY_FROM_DESIRED_THRESHOLD = 0.1;

    private Swerve swerve;
    private Rotation2d[] desiredAngles;

    /**
     * Constructor that really shouldn't need to be used, as Direction based constructor is preferred.
     * 
     * @param swerveSubsystem the swerve subsystem
     * @param angleDegrees the direction to move
     */
    public TurnWheelsForRotation(Swerve swerveSubsystem) {
        super(MAX_SECONDS_TO_TURN_WHEELS);
        this.swerve = swerveSubsystem;
        this.desiredAngles = new Rotation2d[] { Rotation2d.fromDegrees(-135.0),
                                                Rotation2d.fromDegrees(-45.0),
                                                Rotation2d.fromDegrees(135.0),
                                                Rotation2d.fromDegrees(45.0)};
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        swerve.resetAngleEncoders();
        swerve.turnWheelsToAngles(desiredAngles);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
          // Only exit after all are at desired angle, or timer hits timeout
        return (allAtFinalAngles() || super.isFinished());
    }

    private boolean allAtFinalAngles() {
        SwerveModulePosition[] currentPositions = swerve.getPositions();

        for (int i = 0; i <= 3; ++i){
            if (Math.abs(currentPositions[i].angle.getDegrees() - desiredAngles[i].getDegrees()) > DEGRESS_AWAY_FROM_DESIRED_THRESHOLD){
                return false;
            }
        }

        return true;
    } 
}
