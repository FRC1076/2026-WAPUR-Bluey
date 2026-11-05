// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.drum;

public class DrumIODisabled implements DrumIO {
    private double voltageTarget = 0.0;
    private double velocityTargetRadPerSec = 0.0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
        velocityTargetRadPerSec = 0;
    }

    @Override
    public void setVelocity(double radPerSec) {
        velocityTargetRadPerSec = radPerSec;
    }

    @Override
    public void updateInputs(DrumIOInputs inputs) {
        inputs.appliedVoltage = voltageTarget;
        inputs.velocityRadPerSec = velocityTargetRadPerSec;
        inputs.velocityPIDTargetRadPerSec = velocityTargetRadPerSec;
        inputs.PIDActive = velocityTargetRadPerSec != 0;
    }
}
