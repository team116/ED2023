package frc.robot.autos.primitives;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class RotateInPlaceByGyroInDegrees extends SequentialCommandGroup {
    private Swerve swerve;
    private Pigeon2 gyro;
    private double desiredAngleDegrees;
    private double maxPercentPower;
    private static final double DEGREES_AWAY_FROM_DESIRED_THRESHOLD = 0.25;
    private static final double MIN_PERCENT_POWER = 0.05;  // NOTE: This needs to be enough to continue to turn...

    public RotateInPlaceByGyroInDegrees(Swerve swerveSubsystem, double desiredAngleDegrees, double maxPercentPower) {
        this(swerveSubsystem, swerveSubsystem.getGyro(), desiredAngleDegrees, maxPercentPower);
    }

    /**
     * 
     * @param swerveSubsystem
     * @param gyro
     * @param desiredAngleDegrees positive angle degrees are clockwise and negative are counter clockwise
     * @param maxPercentPower
     */
    public RotateInPlaceByGyroInDegrees(Swerve swerveSubsystem, Pigeon2 gyro, double desiredAngleDegrees, double maxPercentPower) {
        this.gyro = gyro;
        this.desiredAngleDegrees = desiredAngleDegrees;
        this.maxPercentPower = maxPercentPower;
        swerve = swerveSubsystem;

        TurnWheelsForRotation turnWheelsForRotation = new TurnWheelsForRotation(swerveSubsystem);
        DriveAtSpeedUntilAngleThresholdReached driveUntilReachAngle = new DriveAtSpeedUntilAngleThresholdReached();

        addCommands(turnWheelsForRotation, driveUntilReachAngle);
    }

    public RotateInPlaceByGyroInDegrees(Swerve swerveSubsystem, double desiredAngleDegrees, RotationDirection rotationDirection, double maxPercentPower) {
        this(swerveSubsystem, desiredAngleDegrees * rotationDirection.getDirectionModifier(), maxPercentPower);
    }

    public RotateInPlaceByGyroInDegrees(Swerve swerveSubsystem, double desiredAngleDegrees, RotationDirection rotationDirection) {
        this(swerveSubsystem, desiredAngleDegrees * rotationDirection.getDirectionModifier(), 0.3);
    }

    private class DriveAtSpeedUntilAngleThresholdReached extends DurationCommand {
        private int atAngleCount;
        private double angleToCheckForInDegrees;

        public DriveAtSpeedUntilAngleThresholdReached() {
            super(2.0);  // FIXME: How long until we give up ??
        }

        @Override
        public void initialize() {
            super.initialize();
            atAngleCount = 0;
            double currentYaw = gyro.getYaw();
            //System.out.println("currentYaw: " + currentYaw);
            //System.out.println("desiredAngleDegrees: " + desiredAngleDegrees);
            angleToCheckForInDegrees = currentYaw - desiredAngleDegrees;  // Assign to computed angle from current angle
            //System.out.println("angle to check: " + angleToCheckForInDegrees);
            SmartDashboard.putNumber("desiredRotationAngle", angleToCheckForInDegrees);   // FIXME: Remove later
        }
    
        @Override
        public void execute() {
            super.execute();

            SmartDashboard.putNumber("currentRotationAngle", gyro.getYaw());  // FIXME: Remove later
            double angleDifference = angleToCheckForInDegrees - gyro.getYaw();
            double absAngleDifference = Math.abs(angleDifference);
            if (Math.abs(absAngleDifference) < DEGREES_AWAY_FROM_DESIRED_THRESHOLD) {
                ++atAngleCount;
                swerve.stop();
            } else {
                //System.out.println("currentAngle: " + gyro.getYaw() + " diff: " + angleDifference);
                double absSpeed = Math.max(Math.min(absAngleDifference / 180.0, maxPercentPower), MIN_PERCENT_POWER);
                swerve.setSpeedPercent(angleDifference < 0.0 ? -absSpeed : absSpeed);
                atAngleCount = 0;
            }
        }
    
        @Override
        public void end(boolean interrupted){
            super.end(interrupted);
            swerve.stop();
        }
    
        @Override
        public boolean isFinished() {
            return (atAngleCount > 3 || super.isFinished());
        }
    }

}
