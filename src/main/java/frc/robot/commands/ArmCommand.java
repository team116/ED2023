package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase{
    private Arm arm;
    private double desiredCanCoderPosition;
    public ArmCommand(Arm armSubSystem){
        this.arm = armSubSystem;
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

        double currentCanCoderPosition = arm.getCANCoderPosition();
        double difference = desiredCanCoderPosition - currentCanCoderPosition;

        SmartDashboard.putNumber("arm CAN Difference", difference);

        double absDifference = Math.abs(difference);
        if (absDifference > 0.75) {
            // cos(-90) is 0 -- aka low vertical  // -75.65 is vertical
            // subtract by 14.45 to get -90
            double cosineVal = Math.abs(Math.cos(Math.toRadians(currentCanCoderPosition - 14.45)));
            double extraPower = (cosineVal * cosineVal) * 0.08;
            SmartDashboard.putNumber("arm CAN Extra", extraPower);
            if (difference > 0) {
                arm.nudgeUp(Math.min((absDifference * 0.015), 0.2) + extraPower);
            } else {
                arm.nudgeDown(/*(absDifference * 0.01)*/ 0.01);
            }
        } else {
            // This is technically insufficient -- will likely need some value to stop it from going down
            arm.stop();
        }
    }

    @Override 
    public void end(boolean interrupted){
        System.out.println("End Default Arm Command");
        arm.stop();
    }
}
