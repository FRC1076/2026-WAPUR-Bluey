package frc.robot.subsystems.pickleballintake;

public class PickleballIntakeIODisabled implements PickleballIntakeIO {
    private double voltageTarget = 0.0;
    private double velocityTarget = 0.0;

    @Override
    public void runVolts(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void setVelocity(double radPerSec) {
        velocityTarget = radPerSec;
    }

    @Override
    public void updateInputs(PickleballIntakeIOInputs inputs) {
        inputs.appliedVolts = voltageTarget;
        inputs.velocityRadPerSec = velocityTarget;
    }

}
