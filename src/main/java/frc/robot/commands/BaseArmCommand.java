package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public abstract class BaseArmCommand extends CommandBase{
    protected Arm arm;
    protected double desiredCanCoderPosition;
    protected boolean moveToDesiredPosition = false;
    protected boolean manualMovementEngaged = false;

    public BaseArmCommand(Arm armSubSystem) {
        this.arm = armSubSystem;

        desiredCanCoderPosition = arm.getCANCoderPosition();
        System.out.println("CONSTRUCTION: " + desiredCanCoderPosition);
        addRequirements(armSubSystem);
    }

    @Override
    public void initialize() {
        desiredCanCoderPosition = arm.getCANCoderPosition();
        moveToDesiredPosition = false;
        manualMovementEngaged = false;
        arm.stop();
        System.out.println("INITIALIZE: " + desiredCanCoderPosition);
    }

    @Override
    public void execute(){
        // Use this default command to keep the arm at the desired position
        SmartDashboard.putNumber("Arm Motor Encoder", arm.getEncoder());
        SmartDashboard.putNumber("arm CAN Desired", desiredCanCoderPosition);
        SmartDashboard.putNumber("arm CAN Coder", arm.getCANCoderPosition());

        checkForDriverInputs();

        if (!manualMovementEngaged) {
            checkForMoveToPositionRequests();

            if (moveToDesiredPosition) {
                moveToDesiredPosition();
            } else {
                holdDesiredPosition();
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("End Base Arm Command");
        arm.stop();
    }

    protected void checkForDriverInputs() {
        // Override this to have manual inputs be handled
        manualMovementEngaged = false;
    }

    protected void checkForMoveToPositionRequests() {
        // Override this to check for new position requests
        moveToDesiredPosition = false;
    }

    private void holdDesiredPosition() { 
        double currentCanCoderPosition = arm.getCANCoderPosition();
        double difference = desiredCanCoderPosition - currentCanCoderPosition;

        SmartDashboard.putNumber("arm CAN Difference", difference);
        double absDifference = Math.abs(difference);

        double holdValue = getHoldValueAtAngle(desiredCanCoderPosition);

        if (absDifference < 0.25) {
            arm.holdWithPower(holdValue);
        } else {
            if (difference > 0) {
                arm.holdWithPower(holdValue + 0.02);
            } else {
                arm.holdWithPower(holdValue - 0.02);
            }
        }
    }

    private void moveToDesiredPosition() {
        double currentCanCoderPosition = arm.getCANCoderPosition();
        double difference = desiredCanCoderPosition - currentCanCoderPosition;

        // 5 -> 0.1
        // 10 -> 0.2
        // 20+ -> 0.4 (0.3)

        SmartDashboard.putNumber("arm CAN Difference", difference);

        double absDifference = Math.abs(difference);
        // cos(-90) is 0 -- aka low vertical  // -75.65 is vertical down
        // subtract by 14.45 to get -90
        double normalizedCanCoderPosition = currentCanCoderPosition - 14.45;

        boolean behindVertical = normalizedCanCoderPosition < -90.0;

        double cosineVal = Math.cos(Math.toRadians(normalizedCanCoderPosition));
        double absCosineVal = Math.abs(cosineVal);
        double feedForward = absCosineVal * 0.03; // Need to find out what percentage is sufficient to hold... 
        // Might fold these values in...

        if (absDifference > 2.0) {
            if (difference > 0) {
                arm.nudgeUp(between((absDifference / 50.0), 0.1, 0.3));
            } else {
                arm.nudgeDown(between(absDifference / 50.0, 0.1, 0.2));
            }
        } else {
            moveToDesiredPosition = false;
        }
    }

    private boolean isBehindVertical(double angle) {
        // cos(-90) is 0 -- aka low vertical  // -75.65 is vertical down
        // subtract by 14.45 to get -90
        double normalizedCanCoderPosition = angle - 14.45;
        return normalizedCanCoderPosition < -90.0;
    }

    private double getHoldValueAtAngle(double angle) {
        // cos(-90) is 0 -- aka low vertical  // -75.65 is vertical down
        // subtract by 14.45 to get -90
        double normalizedCanCoderPosition = angle - 14.45;

        double cosineVal = Math.cos(Math.toRadians(normalizedCanCoderPosition));
        double absCosineVal = Math.abs(cosineVal);

        double holdValue = absCosineVal * 0.09; // NOTE: This is the thing
        return isBehindVertical(angle) ? -holdValue : holdValue;
    }

    private static double between(double value, double min, double max) {
        if (max < min) {
            throw new IllegalArgumentException("Min : " + min + " cannot be larger than max: " + max);
        }

        if (value < min) {
            return min;
        }

        if (value > max) {
            return max;
        }

        return value;
    }

}
