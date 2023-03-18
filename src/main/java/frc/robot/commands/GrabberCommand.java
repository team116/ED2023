package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;

public class GrabberCommand extends CommandBase {

    private Grabber grabber;

    public GrabberCommand(Grabber grabberSubSystem) {
        this.grabber = grabberSubSystem;
        addRequirements(grabberSubSystem);
    }

    @Override
    public void initialize() {
        System.out.println("Begin Default Grabber Command");
        grabber.stop();
    }

    @Override
    public void execute() {
        grabber.slowStallIntake();

        /*
        if (grabber.limitSwitchIsPressed()) {
            grabber.slowStallIntake();
        } else {
            grabber.stop();
        }
        */

    }

    @Override
    public void end(boolean interrupted){
        System.out.println("End Default Grabber Command");
        grabber.stop();
    }
}
