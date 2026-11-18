package frc.robot.subsystems.grabber.left;

/** Fake IO layer used when the robot isn't real (sim or replay). Doesn't touch any hardware. */
public class LeftGrabberIODisabled implements LeftGrabberIO {
    private double voltageTarget = 0.0;
    private double positionTargetRadians = 0.0;
    private double velocityTargetRadPerSec = 0.0;

    @Override
    public void runVolts(double voltage) {
        voltageTarget = voltage;
    }

    @Override
    public void setPosition(double radians) {
        positionTargetRadians = radians;
    }

    @Override
    public void setVelocity(double radPerSec) {
        velocityTargetRadPerSec = radPerSec;
    }

    @Override
    public void updateInputs(LeftGrabberIOInputs inputs) {
        inputs.appliedVolts = voltageTarget;
        inputs.positionRadians = positionTargetRadians;
        inputs.velocityRadPerSec = velocityTargetRadPerSec;
    }
}
