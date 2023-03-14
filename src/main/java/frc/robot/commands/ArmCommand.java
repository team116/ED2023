package frc.robot.commands;

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
        // cos(-90) is 0 -- aka low vertical  // -75.65 is vertical
        // subtract by 14.45 to get -90
        double cosineVal = Math.cos(Math.toRadians(currentCanCoderPosition - 14.45));
        double absCosineVal = Math.abs(cosineVal);

        if (absDifference > 0.75) {
            double nudgeUpFactor = 0.015;
            double nudgeDownFactor = 0.01;
            if (absDifference > 20.0) {
                nudgeUpFactor = 0.2;     // Was 0.4
                nudgeDownFactor = 0.15;  // Was 0.3
            } else if (absDifference > 15.0) {
                nudgeUpFactor = 0.1;     // Was 0.2
                nudgeDownFactor = 0.05;  // Was 0.1
            } else if (absDifference > 8.0) {
                nudgeUpFactor = 0.05;    // Was 0.1
                nudgeDownFactor = 0.01;  // Was 0.5
            }

            double extraPower = (absCosineVal * absCosineVal) * 0.08;
            SmartDashboard.putNumber("arm CAN Extra", extraPower);
            if (difference > 0) {
                arm.nudgeUp(Math.min((absDifference * nudgeUpFactor), 0.2) + extraPower);
            } else {
                arm.nudgeDown(/*(absDifference * 0.01)*/ 0.01);
                //arm.nudgeDown(nudgeDownFactor);
            }
        } else {
            // This is technically insufficient -- will likely need some value to stop it from going down
            arm.stop();
            //arm.holdWithPower(cosineVal * cosineVal * 0.01);
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
        // cos(-90) is 0 -- aka low vertical  // -75.65 is vertical
        // subtract by 14.45 to get -90
        double cosineVal = Math.cos(Math.toRadians(currentCanCoderPosition - 14.45));
        double absCosineVal = Math.abs(cosineVal);
        // Might fold these values in...

        if (absDifference > 2.0) {
            if (difference > 0) {
                arm.nudgeUp(Math.min(0.3, absDifference / 50.0));
            } else {
                arm.nudgeDown(Math.min(0.2, absDifference / 50.0));
            }
        } else {
            moveToDesiredPosition = false;
        }
    }

    private void checkForDriverInputs() {
        double upDownValue = gunnerLogitech.getY();

        double adjustedUpDownValue = upDownValue > 0 ? upDownValue * 0.2 : upDownValue * 0.3;

        if (Math.abs(adjustedUpDownValue) > 0.03) {
            moveToDesiredPosition = false;
            manualMovementEngaged = true;
            arm.move(adjustedUpDownValue);
        } else if (manualMovementEngaged) {  // Previously was doing manual movement, but no longer, so turn it off
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
}
