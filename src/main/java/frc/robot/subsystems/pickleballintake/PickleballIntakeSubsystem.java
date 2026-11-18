package frc.robot.subsystems.pickleballintake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    /** Runs the intake at the given voltage, and stops it when the command ends */
    public Command runVoltage(double volts) {
        return startEnd(() -> io.runVolts(volts), () -> io.runVolts(0.0));
    }

    /** Runs the intake at the given velocity */
    public Command applyVoltage(double volts) {
        return Commands.runOnce(() -> io.runVolts(volts));
    }

    /** Runs the intake at the given velocity, and stops it when the command ends */
    public Command runVelocity(double radiansPerSecond) {
        return startEnd(() -> io.setVelocity(radiansPerSecond), () -> io.runVolts(0.0));
    }

    /** Runs the intake at the given velocity */
    public Command applyVelocity(double volts) {
        return Commands.runOnce(() -> io.setVelocity(volts));
    }

    /** Runs the intake inward while held, and stops it when released */
    public Command intake() {
        return applyVoltage(PickleballIntakeConstants.kIntakeVolts);
    }

    /** Runs the intake backwards while held, and stops it when released */
    public Command reverse() {
        return applyVoltage(PickleballIntakeConstants.kReverseVolts);
    }

    /** Stops the intake */
    public Command stop() {
        return runOnce(() -> io.runVolts(0.0));
    }
}
