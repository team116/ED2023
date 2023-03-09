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
      new JoystickButton(driver, XboxController.Button.kB.value);

  private final JoystickButton armMotorForward = 
      new JoystickButton(driver, XboxController.Button.kY.value);

  private final JoystickButton armMotorReverse = 
      new JoystickButton(driver, XboxController.Button.kA.value);

  private final JoystickButton enableArmLimitSwitches =
      new JoystickButton(driver, XboxController.Button.kX.value);

  private final JoystickButton autoAlignMacroButton =
      new JoystickButton(driver, XboxController.Button.kStart.value);

  private final POVButton dpadUp = new POVButton(driver, 0);
  private final POVButton dpadRight = new POVButton(driver, 90);
  private final POVButton dpadDown = new POVButton(driver, 180);
  private final POVButton dpadLeft = new POVButton(driver, 270);

   /* Subsystems */
  private final Arm arm = new Arm();
  private final Limelight limelight = new Limelight();
  private final Swerve s_Swerve = new Swerve();

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

    arm.setDefaultCommand(new ArmCommand(arm));
    // Configure the button bindings
    configureButtonBindings();

    sendableChooser.addOption("Do Nothing", new DoNothingCommand());
    sendableChooser.setDefaultOption("Drive To Position", new DriveToPositionCommand(s_Swerve));
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
    Trigger armMotorForwardTrigger = armMotorForward.whileTrue(new RepeatCommand(new InstantCommand(() -> arm.moveUp())));
    armMotorForwardTrigger.onFalse(new InstantCommand(() -> arm.stop()));
    Trigger armMotorReverseTrigger = armMotorReverse.whileTrue(new RepeatCommand(new InstantCommand(() -> arm.moveDown())));
    armMotorReverseTrigger.onFalse(new InstantCommand(() -> arm.stop()));

    autoAlignMacroButton.onTrue(new PoleAlignmentCommand(s_Swerve, limelight));

    enableArmLimitSwitches.onTrue(new InstantCommand(() -> arm.disableLimitSwitches()));
    enableArmLimitSwitches.onFalse(new InstantCommand(() -> arm.enableLimitSwitches()));

    // NOTE: These are just debugging examples of possible ways to use dpad
    dpadUp.onTrue(new InstantCommand(() -> arm.moveToPos(Position.HIGH_GOAL)));
    dpadRight.onTrue(new InstantCommand(() -> arm.moveToPos(Position.DRIVE)));
    dpadDown.onTrue(new InstantCommand(() -> arm.moveToPos(Position.PICK_UP)));
    dpadLeft.onTrue(new InstantCommand(() -> arm.moveToPos(Position.MID_GOAL)));

    //dpadUp.whileTrue(new RepeatCommand(new InstantCommand(() -> System.out.println("arm up"))));
    //dpadDown.whileTrue(new RepeatCommand(new InstantCommand(() -> System.out.println("arm down"))));
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
    return shape(start) / 2.0d;
  }
}
