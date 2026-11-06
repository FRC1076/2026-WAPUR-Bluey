package frc.robot.subsystems.elevator;

public class ElevatorIODisabled implements ElevatorIO {

    private double voltageTarget = 0;
    private double targetPosition = 0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVolts = voltageTarget;
        inputs.positionMeters = targetPosition;
    }

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void setPosition(double positionMeters) {
        this.targetPosition = positionMeters;
    }

    @Override
    public void stop() {
        this.setVoltage(0);
    }

}
