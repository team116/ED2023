package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Leds;

public class DefaultLedCommand extends CommandBase {
    private Leds leds;

    public DefaultLedCommand(Leds leds) {
        this.leds = leds;
        addRequirements(this.leds);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            leds.setColor(Leds.Color.RED);
        } else {
            leds.setColor(Leds.Color.BLUE);
        }
    }
}
