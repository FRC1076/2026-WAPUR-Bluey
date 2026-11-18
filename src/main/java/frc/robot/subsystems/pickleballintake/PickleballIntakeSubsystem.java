package frc.robot.subsystems.pickleballintake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PickleballIntakeSubsystem extends SubsystemBase {
    private final PickleballIntakeIO io;
    private final PickleballIntakeIOInputsAutoLogged inputs = new PickleballIntakeIOInputsAutoLogged();

    public final Trigger voltageApplied =
        new Trigger(() -> Math.abs(inputs.appliedVolts) > PickleballIntakeConstants.kVoltageAppliedThreshold);

    public PickleballIntakeSubsystem(PickleballIntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("PickleballIntake", inputs);
    }

    public Command setVoltage(double volts) {
        return startEnd(() -> io.runVolts(volts), () -> io.runVolts(0.0));
    }

    public Command setVelocity(double radiansPerSecond) {
        return startEnd(() -> io.setVelocity(radiansPerSecond), () -> io.runVolts(0.0));
    }
}
