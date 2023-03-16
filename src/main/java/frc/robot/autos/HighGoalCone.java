package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;

/**
 * This should allow for attempting to drive to a specific position
 */
public class HighGoalCone extends CommandBase {
    private static final double CONVERSION_FACTOR = 39.37;
    private Swerve swerve;
    private Arm arm;
    private Grabber grabber;
    private int pidSlot = 1;
    private double distance = -36 / CONVERSION_FACTOR;
    private int step = 0;
    Timer timer;
    private Rotation2d angle = Rotation2d.fromDegrees(0.0d);
    private SwerveModulePosition [] listOfModulePositions = new SwerveModulePosition[4];

    public HighGoalCone(Swerve swerveSubsystem, Arm armSubsystem, Grabber grabberSubsystem){
        swerve = swerveSubsystem;
        arm = armSubsystem;
        grabber = grabberSubsystem;
        swerve.setPID(10, 0, 0, pidSlot);
        swerve.setMinMax(-0.2, 0.2, pidSlot);
        swerve.burnFlash();
        addRequirements(swerveSubsystem, armSubsystem);
    }

    @Override
    public void initialize(){
        System.out.println("Initializing");
        swerve.resetRelativeEncoders();
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute(){
        switch(step){
            case 0:
                System.out.println("Executing step 0");
                if(timer.get() > 2){
                    step++;
                }
                break;
            case 1:
                System.out.println("Executing step 1");
                reset();
                break;  
            case 2:     
                System.out.println("Executing step 2");
                //move arm up to high position
                reset();
                break;
            case 3:
                for (int i = 0; i < listOfModulePositions.length; i++){
                    listOfModulePositions[i] = new SwerveModulePosition(-distance, angle);
                }
                swerve.setModulePositions(listOfModulePositions);
                if(timer.get() > 2){
                    step++;
                }
                break;
            case 4:
                reset();
                break;
            case 5:
                //get rid of the cone
                step++;
                break;
            case 6:
                reset();
                break;
            case 7:
                for (int i = 0; i < listOfModulePositions.length; i++){
                    listOfModulePositions[i] = new SwerveModulePosition(distance, angle);
                }
                swerve.setModulePositions(listOfModulePositions);
                if(timer.get() > 2){
                    step++;
                }
                break;
            case 8:
                //move arm down to driving position
                reset();
                break;
            case 9:
                angle = Rotation2d.fromDegrees(90.0d);
                distance = 0;
                for (int i = 0; i < listOfModulePositions.length; i++){
                    listOfModulePositions[i] = new SwerveModulePosition(distance, angle);
                }
                step++;
            case 10:
                reset();
            case 11:
                
            default:
                System.out.println("default");
                break;
        }
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("end");
    }

    public void reset(){
        timer.reset();
        swerve.resetRelativeEncoders();
        step++;
    }
}
