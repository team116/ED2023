package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;

public class DefaultArmCommand extends BaseArmCommand {
    private Joystick gunnerLogitech;
    private Joystick gunnerStation; // FIXME: Don't believe this ended up being necessary

    public DefaultArmCommand(Arm armSubSystem, Joystick gunnerLogitech, Joystick gunnerStation) {
        super(armSubSystem);
        this.gunnerLogitech = gunnerLogitech;
        this.gunnerStation = gunnerStation;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute(){
        super.execute();
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("End Default Arm Command");
       super.end(interrupted);
    }

    @Override
    protected void checkForDriverInputs() {
        // NOTE: This is airplane mode
        double upDownValue = -gunnerLogitech.getY();
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

    @Override
    protected void checkForMoveToPositionRequests() {

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
            //System.out.println("Requested desired position: " + desiredCanCoderPosition);
            moveToDesiredPosition = true;
        }
    }

    public static double shape(double start) {
        if (start < 0){
          return -(start * start);
        }
        return start * start;
    }
}
