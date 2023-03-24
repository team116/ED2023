package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    private Spark ledController;

    public static enum Color {
        RED(0.61),
        BLUE(0.87),
        YELLOW(0.69),
        PURPLE(0.91);

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
}
