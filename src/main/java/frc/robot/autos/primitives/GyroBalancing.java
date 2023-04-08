package frc.robot.autos.primitives;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.autos.primitives.DriveDistanceAtAngle.Direction;
import frc.robot.subsystems.Swerve;

public class GyroBalancing extends DurationCommand {
    private static final double CRAWL_SPEED = 0.15;
    private static final double MIN_DROP_DIFFERNTIAL = 2.5; // degrees minimum we must have dropped to start counting
    private static final double ZERO_ANGLE_AVERAGE_DEGREE = -1.15;  // -1.4 + -0.9 / 2.0 = -1.15

    private Pigeon2 gyro;
    private Swerve swerve;
    private Direction currentDriveDirection;

    private double lastMaxAngle;
    private int countSeenBelowMaxAngle;

    // WARNING:  This assumes that we're already on the ramp and at some significant incline
    public GyroBalancing(Pigeon2 gyro, Swerve swerve, double maxDurationSeconds, Direction driveDirection) {
        super(maxDurationSeconds);
        this.gyro = gyro;
        this.swerve = swerve;
        this.currentDriveDirection = driveDirection;
    }

    @Override
    public void initialize() {
        super.initialize();
        this.lastMaxAngle = 0.0;
        this.countSeenBelowMaxAngle = 0;
    }

    @Override
    public void execute() {
        swerve.setSpeedPercent(CRAWL_SPEED * currentDriveDirection.directionMultiplier);

        double currentGyroPitch = gyro.getPitch() - ZERO_ANGLE_AVERAGE_DEGREE;
        double absCurrentGyroPitch = Math.abs(currentGyroPitch);
        double absLastMaxAngle = Math.abs(lastMaxAngle);
        if (absCurrentGyroPitch >= absLastMaxAngle) {
            lastMaxAngle = currentGyroPitch;
            countSeenBelowMaxAngle = 0;
        } else if (absLastMaxAngle - absCurrentGyroPitch > MIN_DROP_DIFFERNTIAL) {
            ++countSeenBelowMaxAngle;
        }

        super.execute();
    }

    @Override
    public void end(boolean interrupted){
        // System.out.println("End Intake Grabber Command");
        swerve.stop();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return ((countSeenBelowMaxAngle > 3) || super.isFinished());
    }

}
