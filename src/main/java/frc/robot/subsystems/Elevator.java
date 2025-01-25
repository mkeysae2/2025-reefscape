package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final TalonFX motor;

    private final TrapezoidProfile profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(80, 160)
    );

    private final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public Elevator(TalonFX motor){
        this.motor = motor;
        motor.getConfigurator().apply(
            new Slot0Configs().withKP(1)
        );
    }

    @Override
    public void periodic() {
        setpoint = profile.calculate(0.02, setpoint, goal);
        request.Position = setpoint.position;
        request.Velocity = setpoint.velocity;
        motor.setControl(request);
    }

    public Command goToHeight(double height) {
        return startEnd(
            () -> goal = new TrapezoidProfile.State(height, 0),
            () -> goal = new TrapezoidProfile.State(0, 0));
    }
}
