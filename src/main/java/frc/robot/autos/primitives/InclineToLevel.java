package frc.robot.autos.primitives;

import com.ctre.phoenix.sensors.Pigeon2;

/**
 * Runs until it sees a significant incline angle, and then sees it come back close to level.
 */
public class InclineToLevel extends DurationCommand {

    private static final double MINIMUM_INCLINE_ANGLE_DEGREES = 5.0;
    private static final double MAXIMUM_LEVEL_ANGLE_DEGREES = 2.0;

    boolean seenIncline;
    boolean seenLevel;
    private final Pigeon2 gyro;

    public InclineToLevel(Pigeon2 gyro, double maxDurationSeconds) {
        super(maxDurationSeconds);
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
        super.initialize();
        seenIncline = false;
        seenLevel = false;
    }

    @Override
    public void execute() {
        if (seenIncline) {
            seenLevel = Math.abs(gyro.getPitch()) < MAXIMUM_LEVEL_ANGLE_DEGREES;
        } else {
            seenIncline = Math.abs(gyro.getPitch()) > MINIMUM_INCLINE_ANGLE_DEGREES;
        }

        super.execute();
    }

    @Override
    public void end(boolean interrupted){
        // System.out.println("End Intake Grabber Command");
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return ((seenIncline && seenLevel) || super.isFinished());
    }
}
