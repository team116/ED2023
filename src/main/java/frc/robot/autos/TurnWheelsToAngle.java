package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.Swerve;

public class TurnWheelsToAngle extends DurationCommand {

    private static final double MAX_SECONDS_TO_TURN_WHEELS = 0.5;
    private static final double DEGRESS_AWAY_FROM_DESIRED_THRESHOLD = 0.1;

    private Swerve swerve;
    private Rotation2d desiredAngle;

    /**
     * Constructor that really shouldn't need to be used, as Direction based constructor is preferred.
     * 
     * @param swerveSubsystem the swerve subsystem
     * @param angleDegrees the direction to move
     */
    public TurnWheelsToAngle(Swerve swerveSubsystem, double angleDegrees) {
        super(MAX_SECONDS_TO_TURN_WHEELS);
        this.swerve = swerveSubsystem;
        this.desiredAngle = Rotation2d.fromDegrees(angleDegrees);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        swerve.resetAngleEncoders();
        swerve.turnWheelsToToAngle(desiredAngle);
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
        return (allAtFinalAngle() || super.isFinished());
    }

    private boolean allAtFinalAngle() {
        for (SwerveModulePosition currentPosition : swerve.getPositions()) {
            if (Math.abs(currentPosition.angle.getDegrees() - desiredAngle.getDegrees()) > DEGRESS_AWAY_FROM_DESIRED_THRESHOLD) {
                return false;
            }
        }

        return true;
    } 
}
