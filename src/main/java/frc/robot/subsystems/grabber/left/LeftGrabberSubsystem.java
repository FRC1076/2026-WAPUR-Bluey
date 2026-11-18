package frc.robot.subsystems.grabber.left;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.grabber.GrabberConstants;

public class LeftGrabberSubsystem extends SubsystemBase {
    private final LeftGrabberIO io;
    private final LeftGrabberIOInputsAutoLogged inputs = new LeftGrabberIOInputsAutoLogged();

    public final Trigger voltageApplied =
        new Trigger(() -> Math.abs(inputs.appliedVolts) > GrabberConstants.kVoltageAppliedThreshold);

    public LeftGrabberSubsystem(LeftGrabberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LeftGrabber", inputs);
    }

    /** Runs the grabber at the given voltage, and stops it when the command ends */
    public Command setVoltage(double volts) {
        return startEnd(
            () -> io.runVolts(volts),
            () -> io.runVolts(0.0)
        );
    }

    /** Runs the grabber at the given velocity, and stops it when the command ends */
    public Command setVelocity(double radPerSec) {
        return startEnd(
            () -> io.setVelocity(radPerSec),
            () -> io.runVolts(0.0)
        );
    }

    /** Tells the grabber to go to (and hold) the given position */
    public Command setPosition(double radians) {
        return runOnce(() -> io.setPosition(radians));
    }

    /** Runs the grabber inward while held, and stops it when released */
    public Command intake() {
        return setVoltage(GrabberConstants.kIntakeVolts);
    }

    /** Runs the grabber outward while held, and stops it when released */
    public Command outtake() {
        return setVoltage(GrabberConstants.kOuttakeVolts);
    }

    /** Stops the grabber's motor */
    public Command stop() {
        return runOnce(() -> io.runVolts(0.0));
    }

    public double getPositionRadians() {
        return inputs.positionRadians;
    }

    public double getVelocityRadPerSec() {
        return inputs.velocityRadPerSec;
    }
}
