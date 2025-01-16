package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Rainbow implements LedRoutine {
    private double startingHue = 0.0;
    private final AddressableLEDBuffer buffer;
    private final double increment;

    public Rainbow(AddressableLEDBuffer buffer) {
        this.buffer = buffer;
        increment = 360.0 / buffer.getLength();
    }

    @Override
    public void periodic() {
        for (var i = 0; i < buffer.getLength(); i++) {
            int hue = (int)((i + startingHue) * increment);
            buffer.setHSV(i, hue, 255, 255);
        }
        startingHue += 0.2;
    }
}
