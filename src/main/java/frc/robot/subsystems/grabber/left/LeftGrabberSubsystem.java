package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.grabber.GrabberIO.GrabberIOInputs;

public class GrabberSubsystem extends SubsystemBase {
    private final GrabberIO io;
    private final GrabberIOInputs inputs = new GrabberIOInputs();

    public final Trigger leftVoltageApplied = new Trigger(() -> Math.abs(inputs.leftAppliedVolts) > 0.0);
    public final Trigger rightVoltageApplied = new Trigger(() -> Math.abs(inputs.rightAppliedVolts) > 0.0);

    public GrabberSubsystem(GrabberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    private void applyVoltage(double leftVolts, double rightVolts) {
        io.leftRunVolts(leftVolts);
        io.rightRunVolts(rightVolts);
    }

    public Command setVoltage(double leftVolts, double rightVolts) {
        return startEnd(
            () -> applyVoltage(leftVolts, rightVolts),
            () -> applyVoltage(0.0, 0.0)
        );
    }
}
