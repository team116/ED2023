// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.Position;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick gunnerStation = new Joystick(1);
  private final Joystick gunnerLogitech = new Joystick(2);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  private final JoystickButton toggleTesterButton =
      new JoystickButton(driver, XboxController.Button.kX.value);

  // private final JoystickButton armMotorForward =
  //     new JoystickButton(driver, XboxController.Button.kY.value);

  // private final JoystickButton armMotorReverse =
  //     new JoystickButton(driver, XboxController.Button.kA.value);

  private final JoystickButton enableArmLimitSwitches =
      new JoystickButton(driver, XboxController.Button.kStart.value);

  private final JoystickButton autoAlignMacroButton =
      new JoystickButton(driver, XboxController.Button.kB.value);

  private final JoystickButton toggleSlowModeButton =
      new JoystickButton(driver, XboxController.Button.kY.value);

  private final JoystickButton gunnerIntakeButton = new JoystickButton(gunnerStation, 1);
  private final JoystickButton gunnerOutakeButton = new JoystickButton(gunnerStation, 2);

  private final CommandXboxController driverXBoxController = new CommandXboxController(Constants.DRIVER_XBOX_CONTROLLER_PORT);

  private final Trigger driverLeftTrigger = driverXBoxController.leftTrigger(0.5);
  private final Trigger driverRightTrigger = driverXBoxController.rightTrigger(0.5);

  private final POVButton dpadUp = new POVButton(driver, 0);
  private final POVButton dpadRight = new POVButton(driver, 90);
  private final POVButton dpadDown = new POVButton(driver, 180);
  private final POVButton dpadLeft = new POVButton(driver, 270);

   /* Subsystems */
  private final Arm arm = new Arm();
  private final Limelight limelight = new Limelight();
  private final Swerve s_Swerve = new Swerve();
  private final Grabber grabber = new Grabber();

  private final SendableChooser<Command> sendableChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> shape(-driver.getRawAxis(translationAxis)),
            () -> shape(-driver.getRawAxis(strafeAxis)),
            () -> rotationShape(-driver.getRawAxis(rotationAxis)),
            () -> robotCentric.getAsBoolean()));

    limelight.setDefaultCommand(new DefaultLimelightCommand(limelight));

    arm.setDefaultCommand(new ArmCommand(arm, gunnerLogitech, gunnerStation));

    grabber.setDefaultCommand(new GrabberCommand(grabber));

    // Configure the button bindings
    configureButtonBindings();

    sendableChooser.addOption("Do Nothing", new DoNothingCommand());
    sendableChooser.setDefaultOption("Drive To Position", new HighGoalCone(s_Swerve, arm, grabber));
    SmartDashboard.putData(sendableChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    toggleTesterButton.onTrue(new InstantCommand(() -> limelight.toggleStreamMode()));
    autoAlignMacroButton.onTrue(new PoleAlignmentCommand(s_Swerve, limelight));

    //armMotorForward.onTrue(new InstantCommand(() -> arm.moveUp()));
    //armMotorReverse.onTrue(new InstantCommand(() -> arm.moveDown()));
    //Trigger armMotorForwardTrigger = armMotorForward.whileTrue(new RepeatCommand(new InstantCommand(() -> arm.moveUp(), arm)));
    //armMotorForwardTrigger.onFalse(new InstantCommand(() -> arm.stop(), arm));
    //Trigger armMotorReverseTrigger = armMotorReverse.whileTrue(new RepeatCommand(new InstantCommand(() -> arm.moveDown(), arm)));
    //armMotorReverseTrigger.onFalse(new InstantCommand(() -> arm.stop(), arm));

    Trigger armForwardTrigger = dpadUp.whileTrue(new RepeatCommand(new InstantCommand(() -> arm.moveUp(), arm)));
    armForwardTrigger.onFalse(new InstantCommand(() -> arm.stop(), arm));
    Trigger armReverseTrigger = dpadDown.whileTrue(new RepeatCommand(new InstantCommand(() -> arm.moveDown(), arm)));
    armReverseTrigger.onFalse(new InstantCommand(() -> arm.stop(), arm));

    enableArmLimitSwitches.onTrue(new InstantCommand(() -> arm.disableLimitSwitches()));
    enableArmLimitSwitches.onFalse(new InstantCommand(() -> arm.enableLimitSwitches()));

    //dpadUp.whileTrue(new RepeatCommand(new InstantCommand(() -> System.out.println("arm up"))));
    //dpadDown.whileTrue(new RepeatCommand(new InstantCommand(() -> System.out.println("arm down"))));
    driverLeftTrigger.whileTrue(new RepeatCommand(new InstantCommand(() -> grabber.getRidOfGamePiece(), grabber)));
    driverRightTrigger.whileTrue(new RepeatCommand(new InstantCommand(() -> grabber.intakeGamePiece(), grabber)));

    gunnerOutakeButton.whileTrue(new RepeatCommand(new InstantCommand(() -> grabber.getRidOfGamePiece(), grabber)));
    gunnerIntakeButton.whileTrue(new RepeatCommand(new InstantCommand(() -> grabber.intakeGamePiece(), grabber)));

    toggleSlowModeButton.onTrue(new InstantCommand(() -> s_Swerve.toggleSlowMode()));

    // XBox
    // Y for slow mode
    // Right trigger - inwards
    // Left trigger - outwards

    // Logitech
    // Raw 1 - Trigger - Stowed in robot ()
    // Raw 2 - Human Intake

    // y-axis -> "up"/"down"   (invert, as negative)

    // Raw 7  - Cone High
    // Raw 8  - Cube High
    // Raw 9  - Cone Mid
    // Raw 10 - Cube Mid
    // Raw 11 - Low Score
    // Raw 12 - Floor Intake 


    // Gunner
    // Raw 1 - Intake
    // Raw 2 - Outake

    // Raw 3 - Claw Limit Switch Disable (if "on")
    // Raw 4 - Arm Limit Switch Disable (if "on")
    // Raw 5 - Disable all arm operations

    // Raw 7 - Cube/Color Switch  (RGB somewhere)
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new exampleAuto(s_Swerve);
    return sendableChooser.getSelected();
  }

  public static double shape(double start){
    if (start < 0){
      return -(start * start);
    }
    return start * start;
  }

  public static double rotationShape(double start) {
    return shape(start);  // was original dividing by 2 */ / 2.0d;
  }
}
