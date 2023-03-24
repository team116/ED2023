package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Leds;

public class DefaultLedCommand extends CommandBase {
    private Leds leds;
    private Joystick gunnerLogitech;
    private Joystick gunnerStation;
    private Alliance alliance;

    public DefaultLedCommand(Leds leds, Joystick gunnerLogitech, Joystick gunnerStation) {
        this.leds = leds;
        this.gunnerLogitech = gunnerLogitech;
        this.gunnerStation = gunnerStation;
        addRequirements(this.leds);
    }

    @Override
    public void initialize() {
        alliance = DriverStation.getAlliance();
    }

    @Override
    public void execute() {
        if (gunnerLogitech.getRawButtonPressed(6)) {  // FIXME: Might want as gunner station switch to "hold"
            setLedsToDesiredElementColor();
        } else {
            setLedsToAllianceColor();
        }
    }

    private void setLedsToDesiredElementColor() {
        if (gunnerStation.getRawButton(7)) {  // FIXME: Button 3, 5, or something else?  [1,2,4] already in use
            leds.setColor(Leds.Color.YELLOW);
        } else {
            leds.setColor(Leds.Color.PURPLE);
        }
    }

    private void setLedsToAllianceColor() {
        if (alliance == DriverStation.Alliance.Red) {
            leds.setColor(Leds.Color.RED);
        } else {
            leds.setColor(Leds.Color.BLUE);
        }
    }
}
