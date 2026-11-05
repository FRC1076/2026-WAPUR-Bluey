// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.elevator;

public class ElevatorIODisabled implements ElevatorIO {
    private double voltageTarget = 0;
    private double positionTarget = 0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void setVoltageNoSoftStops(double volts) {
        setVoltage(volts);
    }

    @Override
    public void setPosition(double meters) {
        positionTarget = meters;
    }

    @Override
    public void rezero() {
        positionTarget = 0;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVoltage = voltageTarget;
        inputs.heightMeters = positionTarget;
        inputs.PIDTargetMeters = positionTarget;
    }
}
