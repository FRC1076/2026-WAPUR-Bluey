// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.grabber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {
    private final GrabberIO io;
    private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

    public GrabberSubsystem(GrabberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Grabber", inputs);
    }

    /** Set the grabber motors to the specified voltage */
    public Command applyVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    /** Run the grabber motors at the supplied voltage */
    public Command runVoltage(DoubleSupplier volts) {
        return Commands.run(
            () -> io.setVoltage(volts.getAsDouble()),
            this
        );
    }

    /** Set the grabber to the specified velocity */
    public Command applyVelocity(double radPerSec) {
        return Commands.runOnce(
            () -> io.setVelocity(radPerSec),
            this
        );
    }

    /** Run the grabber at the supplied velocity */
    public Command runVelocity(DoubleSupplier radPerSec) {
        return Commands.run(
            () -> io.setVelocity(radPerSec.getAsDouble()),
            this
        );
    }

    /** Run the grabber inwards to grab a game piece */
    public Command grab() {
        return applyVoltage(GrabberConstants.kIntakeVolts);
    }

    /** Apply a small inward voltage to keep hold of a game piece */
    public Command hold() {
        return applyVoltage(GrabberConstants.kHoldVolts);
    }

    /** Run the grabber outwards to eject a game piece */
    public Command eject() {
        return applyVoltage(GrabberConstants.kEjectVolts);
    }

    /** Stop the grabber's motors */
    public Command stop() {
        return applyVoltage(0);
    }

    /** Gets the grabber lead motor's voltage */
    public double getVoltage() {
        return inputs.leadMotorAppliedVoltage;
    }

    /** Gets the grabber lead motor's velocity */
    public double getVelocityRadPerSec() {
        return inputs.leadMotorVelocityRadPerSec;
    }
}
