// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;
        public double heightMeters = 0;

        public double velocityMetersPerSecond = 0;
        public double motorTempDegC = 0;

        public double PIDTargetMeters = 0;
        public boolean PIDActive = false;
    }

    public abstract void setVoltage(double volts);

    public abstract void setVoltageNoSoftStops(double volts);

    public abstract void setPosition(double meters);

    public abstract void updateInputs(ElevatorIOInputs inputs);

    public abstract void rezero();
}
