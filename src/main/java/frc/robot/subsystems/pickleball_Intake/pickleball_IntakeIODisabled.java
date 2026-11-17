package frc.robot.subsystems.pickleball_Intake;

public class pickleball_IntakeIODisabled implements pickleball_IntakeIO {
    private double voltageTarget = 0.0;
    private double velocityTarget = 0.0;

    @Override
    public void runVolts(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void setRPS(double RPS) {
        velocityTarget = RPS;
    }

    @Override
    public void updateInputs(pickleball_IntakeIOInputs inputs) {
        inputs.appliedVolts = voltageTarget;
        inputs.RPS = velocityTarget;
    }

}