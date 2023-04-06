package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void initialize() {
    s_Swerve.resetRelativeEncoders();
  }

  @Override
  public void execute() {

    // Wants a toggle for slow mode/or hold button... someone matters
    // Can do it either way...
    double rawTranslationVal = translationSup.getAsDouble();
    double rawStrafeVal = strafeSup.getAsDouble();
    double rawRotationVal = rotationSup.getAsDouble();

    double maxAngularVelocity = Constants.Swerve.MAX_ANGULAR_VELOCITY_FAST_MODE;
    if (s_Swerve.inSlowMode()) {
      rawTranslationVal *= 0.8;
      rawStrafeVal *= 0.8;
      //rawRotationVal *= 0.5;
      maxAngularVelocity = Constants.Swerve.MAX_ANGULAR_VELOCITY_SLOW_MODE;
    }
    // FIXME: Make slow-mode capability
    // Need to read state from somewhere... s_Swerve maintain state?  Probably...

    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(rawTranslationVal, Constants.Swerve.STICK_DEADBAND));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(rawStrafeVal, Constants.Swerve.STICK_DEADBAND));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(rawRotationVal, Constants.Swerve.STICK_DEADBAND));

    /* Drive */
    /*
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
        rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY,
        !robotCentricSup.getAsBoolean(),
        true);
    */

    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
        rotationVal * maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true);

  }
}
