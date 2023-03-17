package frc.robot.autos;

import frc.robot.subsystems.Grabber;

public class GrabberExpelCommand extends DurationCommand {

    private static final double MAX_EXPEL_SECONDS = 0.5;

    private final Grabber grabber;

    public GrabberExpelCommand(Grabber grabber) {
        super(MAX_EXPEL_SECONDS);
        this.grabber = grabber;
    }

    @Override
    public void initialize() {
        grabber.getRidOfGamePiece();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("End Expel Grabber Command");
        grabber.stop();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

}
