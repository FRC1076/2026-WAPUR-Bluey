package frc.robot.subsystems.kicker;

import java.util.Objects;
import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Command-based control for a voltage-driven kicker motor. */
public class KickerSubsystem extends SubsystemBase {
    private final DoubleConsumer voltageOutput;

    public KickerSubsystem(DoubleConsumer voltageOutput) {
        this.voltageOutput = Objects.requireNonNull(voltageOutput);
    }

    public Command setVoltage(double volts) {
        return startEnd(
            () -> voltageOutput.accept(volts),
            () -> voltageOutput.accept(0.0)
        );
    }
}
