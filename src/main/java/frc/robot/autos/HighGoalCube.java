package frc.robot.autos;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;

public class HighGoalCube extends CommandBase{
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

    public HighGoalCube(Swerve swerveSubsystem){
        swerve = swerveSubsystem;
        swerve.setPID(10, 0, 0, pidSlot);
        swerve.setMinMax(-0.2, 0.2, pidSlot);
        swerve.burnFlash();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerve.resetRelativeEncoders();
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
        switch(step){
            case 0:
                // System.out.println("Executing step 0");
                for (int i = 0; i < listOfModulePositions.length; i++){
                    listOfModulePositions[i] = new SwerveModulePosition(distance, angle);
                }
                swerve.setModulePositions(listOfModulePositions);
                if(timer.get() > 2){
                    step++;
                }
                break;
            case 1:
                // System.out.println("Executing step 1");
                timer.reset();
                step++;
                break;  
            case 2:     
                // System.out.println("Executing step 2");
                //move arm up to high position
                step++;
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
                timer.reset();
                step++;
                break;
            case 5:
                //get rid of the cone
                step++;
                break;
            case 6:
                timer.reset();
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
                break;
            default:
                // System.out.println("default");
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
