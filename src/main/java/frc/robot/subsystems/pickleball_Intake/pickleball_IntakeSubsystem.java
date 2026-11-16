package frc.robot.subsystems.pickleball_Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.pickleball_Intake.pickleball_IntakeIO.pickleball_IntakeIOInputs;

public class pickleball_IntakeSubsystem extends SubsystemBase {
    private final pickleball_IntakeIO io;
    private final pickleball_IntakeIOInputs inputs = new pickleball_IntakeIOInputs();

    public final Trigger voltageApplied = new Trigger(() -> Math.abs(inputs.appliedVolts) > 0.0);

    public pickleball_IntakeSubsystem(pickleball_IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command setVoltage(double volts) {
        return startEnd(() -> io.runVolts(volts), () -> io.runVolts(0.0));
    }

    public Command setVelocity(double radiansPerSecond) {
        return startEnd(() -> io.setRPS(radiansPerSecond), () -> io.runVolts(0.0));
    }
}
