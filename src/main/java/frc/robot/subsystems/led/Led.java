package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    public Led(int port, int length) {
        leds = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        leds.setLength(buffer.getLength());
        leds.start();

    }

    @Override
    public void periodic() {
        leds.setData(buffer);
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(buffer));
    }
}
