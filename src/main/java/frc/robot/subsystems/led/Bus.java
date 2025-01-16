package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class Bus implements LedRoutine {
    private final AddressableLEDBuffer buffer;
    private int pos = 0;
    private int increment = 1;


    public Bus(AddressableLEDBuffer buffer) {
        this.buffer = buffer;

    }

    @Override
    public void periodic() {
        int newPos = pos + increment;

        if (newPos >= buffer.getLength() || newPos < 0) {
            increment = -increment;
            newPos += 2 * increment;
        }

        buffer.setLED(newPos, Color.kRed);
        buffer.setLED(pos, Color.kBlack);
        pos = newPos;
    }

}
