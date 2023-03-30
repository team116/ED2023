package frc.robot.autos.primitives;

import frc.robot.subsystems.Grabber;

public class GrabberIntakeCommand extends DurationCommand {

    private static final double MAX_INTAKE_WAIT_SECONDS = 3.0;

    private int stabilizedCount;
    private final Grabber grabber;

    public GrabberIntakeCommand(Grabber grabber) {
        super(MAX_INTAKE_WAIT_SECONDS);
        this.grabber = grabber;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        super.initialize();
        grabber.intakeGamePiece();
        stabilizedCount = 0;
    }

    @Override
    public void execute() {
        if (grabber.limitSwitchIsPressed()) {
            grabber.slowStallIntake();
            ++stabilizedCount;
        } else {
            grabber.intakeGamePiece();
            stabilizedCount = 0;
        }

        super.execute();
    }

    @Override
    public void end(boolean interrupted){
        // System.out.println("End Intake Grabber Command");
        grabber.stop();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return (stabilizedCount > 3 || super.isFinished());
    }

}
