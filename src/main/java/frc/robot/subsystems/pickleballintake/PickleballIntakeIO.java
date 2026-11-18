package frc.robot.subsystems.pickleballintake;

import org.littletonrobotics.junction.AutoLog;

public interface PickleballIntakeIO {
    @AutoLog
    public static class PickleballIntakeIOInputs {
        public double appliedVolts = 0;
        public double currentAmps = 0;
        public double positionRadians = 0;
        public double velocityRadPerSec = 0;
        public double motorTempDegC = 0;
    }

    public abstract void updateInputs(PickleballIntakeIOInputs inputs);

    public abstract void runVolts(double voltage);

    public abstract void setVelocity(double radPerSec);
}
