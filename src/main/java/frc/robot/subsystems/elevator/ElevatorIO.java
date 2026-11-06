package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    public static record ElevatorControlConstants (
        Double kP,
        Double kI,
        Double kD,
        Double kS,
        Double kG,
        Double kV,
        Double kA
    ) {}

    @AutoLog
    public static class ElevatorIOInputs {

        public double appliedVolts = 0;
        public double currentAmps = 0;
        public double statorCurrentAmps = 0;
        public double motorTempDegreesC = 0;

        public double positionMeters = 0;
        public double velocityMetersPerSecond = 0;
    }

    public abstract void updateInputs(ElevatorIOInputs inputs);
    public abstract void setVoltage(double volts);
    public abstract void setPosition(double positionMeters);
    public abstract void stop();
    public default void simulationPeriodic() {}

}