package frc.robot.subsystems.grabber.right;

import org.littletonrobotics.junction.AutoLog;

public interface RightGrabberIO {

    @AutoLog
    public static class RightGrabberIOInputs {
        public double appliedVolts = 0;
        public double currentAmps = 0;
        public double positionRadians = 0;
        public double velocityRadPerSec = 0;
        public double motorTempDegC = 0;
    }

    // updates inputs for the grabber
    public abstract void updateInputs(RightGrabberIOInputs inputs);

    // changes voltage of the motor
    public abstract void runVolts(double voltage);

    // sets postion of the grabber
    public abstract void setPosition(double radians);

    // sets velocity of the motor
    public abstract void setVelocity(double radPerSec);
}
