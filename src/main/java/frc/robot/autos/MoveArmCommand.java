package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.BaseArmCommand;
import frc.robot.subsystems.Arm;

public class MoveArmCommand extends BaseArmCommand {

    private static final double MAX_SECONDS_TO_WAIT = 3.0;
    private static final double DEGREES_AWAY_FROM_DESIRED_THRESHOLD = 0.5;

    private int stabilizedCount;
    private final Arm.Position desiredArmPosition;
    private double startTime;

    public MoveArmCommand(Arm armSubSystem, Arm.Position desiredArmPosition) {
        super(armSubSystem);
        this.desiredArmPosition = desiredArmPosition;
    }

    @Override
    public void initialize() {
        super.initialize();
        desiredCanCoderPosition = desiredArmPosition.getAngleDegrees();
        moveToDesiredPosition = true;
        stabilizedCount = 0;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        super.execute();

        if (Math.abs(arm.getCANCoderPosition() - desiredCanCoderPosition) < DEGREES_AWAY_FROM_DESIRED_THRESHOLD) {
            ++stabilizedCount;
        } else {
            stabilizedCount = 0;
        }

    }

    @Override
    public void end(boolean interrupted){
       super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return (stabilizedCount > 3 || (Timer.getFPGATimestamp() - startTime > MAX_SECONDS_TO_WAIT));
    }
}
