// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

public interface GrabberIO {
    @AutoLog
    public static class GrabberIOInputs {
        public double leadMotorAppliedVoltage = 0;
        public double followMotorAppliedVoltage = 0;

        public double leadMotorCurrentAmps = 0;
        public double followMotorCurrentAmps = 0;

        public double leadMotorVelocityRadPerSec = 0;
        public double followMotorVelocityRadPerSec = 0;

        public double leadMotorTempDegC = 0;
        public double followMotorTempDegC = 0;

        public double velocityPIDTargetRadPerSec = 0;
        public boolean PIDActive = false;
    }

    public abstract void setVoltage(double volts);

    public abstract void setVelocity(double radPerSec);

    public abstract void updateInputs(GrabberIOInputs inputs);
}
