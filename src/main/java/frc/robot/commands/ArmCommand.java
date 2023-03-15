package frc.robot.commands;

import java.util.concurrent.CountDownLatch;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase{
    private Arm arm;
    private double desiredCanCoderPosition;
    private Joystick gunnerLogitech;
    private Joystick gunnerStation; // FIXME: Don't believe this ended up being necessary
    private boolean moveToDesiredPosition = false;
    private boolean manualMovementEngaged = false;

    public ArmCommand(Arm armSubSystem, Joystick gunnerLogitech, Joystick gunnerStation) {
        this.arm = armSubSystem;
        this.gunnerLogitech = gunnerLogitech;
        this.gunnerStation = gunnerStation;

        desiredCanCoderPosition = arm.getCANCoderPosition();
        addRequirements(armSubSystem);
    }

    @Override
    public void initialize() {
        desiredCanCoderPosition = arm.getCANCoderPosition();
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
        System.out.println("End Default Arm Command");
        arm.stop();
    }

    private void holdDesiredPosition() { 
        double currentCanCoderPosition = arm.getCANCoderPosition();
        double difference = desiredCanCoderPosition - currentCanCoderPosition;

        SmartDashboard.putNumber("arm CAN Difference", difference);
        double absDifference = Math.abs(difference);

        double holdValue = getHoldValueAtAngle(desiredCanCoderPosition);

        if (absDifference < 0.75) {
            arm.holdWithPower(holdValue);
        } else {
            if (difference > 0) {
                arm.holdWithPower(holdValue + 0.02);
            } else {
                arm.holdWithPower(holdValue - 0.01);
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

    boolean isBehindVertical(double angle) {
        // cos(-90) is 0 -- aka low vertical  // -75.65 is vertical down
        // subtract by 14.45 to get -90
        double normalizedCanCoderPosition = angle - 14.45;
        return normalizedCanCoderPosition < -90.0;
    }

    double getHoldValueAtAngle(double angle) {
        // cos(-90) is 0 -- aka low vertical  // -75.65 is vertical down
        // subtract by 14.45 to get -90
        double normalizedCanCoderPosition = angle - 14.45;

        double cosineVal = Math.cos(Math.toRadians(normalizedCanCoderPosition));
        double absCosineVal = Math.abs(cosineVal);

        double holdValue = absCosineVal * 0.09; // NOTE: This is the thing
        return isBehindVertical(angle) ? -holdValue : holdValue;
    }

    private void checkForDriverInputs() {
        double upDownValue = gunnerLogitech.getY();
        upDownValue = shape(upDownValue);

        double adjustedUpDownValue = upDownValue > 0 ? upDownValue * 0.3 : upDownValue * 0.4;  // FIXME: Might even go faster than this

        SmartDashboard.putNumber("arm Manual raw value", upDownValue);
        SmartDashboard.putNumber("arm Manual value", adjustedUpDownValue);
        if (Math.abs(adjustedUpDownValue) > 0.03) {
            SmartDashboard.putString("arm Manual In Process", "true");
            moveToDesiredPosition = false;
            manualMovementEngaged = true;
            arm.move(adjustedUpDownValue);
        } else if (manualMovementEngaged) {  // Previously was doing manual movement, but no longer, so turn it off
            SmartDashboard.putString("arm Manual In Process", "false");
            manualMovementEngaged = false;
            arm.stop();
            desiredCanCoderPosition = arm.getCANCoderPosition();
        }
    }

    private void checkForMoveToPositionRequests() {

        Arm.Position desiredArmPosition = null;

        if (gunnerLogitech.getRawButtonPressed(1)) {
            desiredArmPosition = Arm.Position.STOWED;
        } else if (gunnerLogitech.getRawButtonPressed(2)) {
            desiredArmPosition = Arm.Position.HUMAN_PLAYER_STATION;
        } else if (gunnerLogitech.getRawButtonPressed(7)) {
            desiredArmPosition = Arm.Position.CONE_HIGH_GOAL;
        } else if (gunnerLogitech.getRawButtonPressed(8)) {
            desiredArmPosition = Arm.Position.CUBE_HIGH_GOAL;
        } else if (gunnerLogitech.getRawButtonPressed(9)) {
            desiredArmPosition = Arm.Position.CONE_MID_GOAL;
        } else if (gunnerLogitech.getRawButtonPressed(10)) {
            desiredArmPosition = Arm.Position.CUBE_MID_GOAL;
        } else if (gunnerLogitech.getRawButtonPressed(11)) {
            desiredArmPosition = Arm.Position.LOW_GOAL;
        } else if (gunnerLogitech.getRawButtonPressed(12)) {
            desiredArmPosition = Arm.Position.FLOOR_INTAKE;
        }

        // Raw 1 - Trigger - Stowed in robot ()
        // Raw 2 - Human Intake
      
        // y-axis -> "up"/"down"   (invert, as negative)
      
        // Raw 7  - Cone High
        // Raw 8  - Cube High
        // Raw 9  - Cone Mid
        // Raw 10 - Cube Mid
        // Raw 11 - Low Score
        // Raw 12 - Floor Intake

        if (desiredArmPosition != null) {
            desiredCanCoderPosition = desiredArmPosition.getAngleDegrees();
            moveToDesiredPosition = true;
        }
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

    public static double shape(double start) {
        if (start < 0){
          return -(start * start);
        }
        return start * start;
    }
}
