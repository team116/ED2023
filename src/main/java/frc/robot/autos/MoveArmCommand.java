package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.BaseArmCommand;
import frc.robot.subsystems.Arm;

public class MoveArmCommand extends BaseArmCommand {

    private static final double DEFAULT_MAX_SECONDS_TO_WAIT = 4.0;
    private static final double DEGREES_AWAY_FROM_DESIRED_THRESHOLD = 1.0;

    private int stabilizedCount;
    private final Arm.Position desiredArmPosition;
    private double startTime;
    private final double timeToWaitSeconds;
    private final HoldArmCommand holdArmCommand;

    // FIXME: THIS SHOULD NOT BE USED IN THE FUTURE, TAKE IT OUT!!!!!!
    public MoveArmCommand(Arm armSubSystem, Arm.Position desiredArmPosition) {
        this(armSubSystem, desiredArmPosition, DEFAULT_MAX_SECONDS_TO_WAIT);
    }
   
    public MoveArmCommand(Arm armSubSystem, Arm.Position desiredArmPosition, double maxWaitSeconds) {
        this(armSubSystem, desiredArmPosition, maxWaitSeconds, null);
    }
    // FIXME: Both of the above constructors need to be removed, only leaving to not change billizions of code yet

    public MoveArmCommand(Arm armSubSystem, Arm.Position desiredArmPosition, HoldArmCommand holdArmCommand) {
        this(armSubSystem, desiredArmPosition, DEFAULT_MAX_SECONDS_TO_WAIT, holdArmCommand);
    }

    public MoveArmCommand(Arm armSubSystem, Arm.Position desiredArmPosition, double maxWaitSeconds, HoldArmCommand holdArmCommand) {
        super(armSubSystem);
        this.desiredArmPosition = desiredArmPosition;
        timeToWaitSeconds = maxWaitSeconds;
        this.holdArmCommand = holdArmCommand;
    }

    @Override
    public void initialize() {
        super.initialize();
        desiredCanCoderPosition = desiredArmPosition.getAngleDegrees();
        moveToDesiredPosition = true;
        stabilizedCount = 0;
        startTime = Timer.getFPGATimestamp();
        if (holdArmCommand != null) {
            holdArmCommand.disable();
        }
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
        // System.out.println("Auto lift arm exited???");
        super.end(interrupted);
        if (holdArmCommand != null) {
            holdArmCommand.enable();
        }
    }

    @Override
    public boolean isFinished() {
        return (stabilizedCount > 3 || (Timer.getFPGATimestamp() - startTime > timeToWaitSeconds));
    }
}
