package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    private Spark ledController;
    private boolean enabled;

    public static enum Color {
        RED(0.61),
        RED_LIGHT_CHASE(-0.31),
        RED_HEARTBEAT(-0.25),
        RED_BREATH(-0.17),
        RED_SHOT(-0.85),
        BLUE_LIGHT_CHASE(-0.29),
        BLUE_HEARTBEAT(-0.23),
        BLUE_BREATH(-0.15),
        BLUE_SHOT(-0.83),
        BLUE(0.87),
        YELLOW(0.69),
        PURPLE(0.91),
        PURPLE_BLUE(0.89)
        ;

        private double colorSpeed;

        private Color(double colorSpeed) {
            this.colorSpeed = colorSpeed;
        }
    }

    public Leds() {
        ledController = new Spark(0);
    }

    public void setColor(Color color) {
        ledController.set(color.colorSpeed);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void enable() {
        setEnabled(true);
    }

    public void disable() {
        setEnabled(false);
    }
}
