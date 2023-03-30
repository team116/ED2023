package frc.robot.autos;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.DriveDistanceAtAngle.Speed;
import frc.robot.commands.PoleAlignmentCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.subsystems.Arm.Position.*;
import static frc.robot.autos.DriveDistanceAtAngle.Direction.*;

public class PickUpSecondPieceAfterHighConeAndScoreInGroundGoal extends HighGoalCone{
    public PickUpSecondPieceAfterHighConeAndScoreInGroundGoal(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem, Limelight limelight, Pigeon2 gyro){
        super(swerveSubsystem, armSubsystem, grabberSubsystem, limelight);
        HoldArmCommand holdArmCommand = new HoldArmCommand(armSubsystem);

        RotateInPlaceByGyroInDegrees rotate180 = new RotateInPlaceByGyroInDegrees(swerveSubsystem, gyro, 180, 0.1);

        DriveDistanceAtAngle moveLeft = new DriveDistanceAtAngle(swerveSubsystem, 4, LEFT);

        ParallelCommandGroup driveForwardsAndIntakeCone = new ParallelCommandGroup(
            new DriveDistanceAtAngle(swerveSubsystem, 4, FORWARD), 
            new GrabberIntakeCommand(grabberSubsystem));

        DriveDistanceAtAngle moveRight = new DriveDistanceAtAngle(swerveSubsystem, 4, RIGHT);

        RotateInPlaceByGyroInDegrees rotateback180 = new RotateInPlaceByGyroInDegrees(swerveSubsystem, gyro, 0, 0.1);

        DriveDistanceAtAngle moveBackToGroundGoal = new DriveDistanceAtAngle(swerveSubsystem, 220.0, FORWARD);

        GrabberExpelCommand scoreConeInGround = new GrabberExpelCommand(grabberSubsystem);

        SequentialCommandGroup internalCommandGroup = new SequentialCommandGroup(
            rotate180,
            moveLeft,
            driveForwardsAndIntakeCone,
            moveRight,
            rotateback180,
            moveBackToGroundGoal,
            scoreConeInGround,
            new InstantCommand(() -> holdArmCommand.killIt()));

        ParallelCommandGroup holdArmAndOthers = new ParallelCommandGroup(holdArmCommand, internalCommandGroup);

        addCommands(holdArmAndOthers);
    
    }
}
