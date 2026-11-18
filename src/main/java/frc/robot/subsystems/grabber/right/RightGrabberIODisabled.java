package frc.robot.subsystems.grabber.right;

/** Fake IO layer used when the robot isn't real (sim or replay). Doesn't touch any hardware. */
public class RightGrabberIODisabled implements RightGrabberIO {
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
    public void updateInputs(RightGrabberIOInputs inputs) {
        inputs.appliedVolts = voltageTarget;
        inputs.positionRadians = positionTargetRadians;
        inputs.velocityRadPerSec = velocityTargetRadPerSec;
    }
}
