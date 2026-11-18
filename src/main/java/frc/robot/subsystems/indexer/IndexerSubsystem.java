// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.indexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    /** Set the motor to the specified voltage. Keeps running after the command ends! */
    public Command applyVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    /** Run the motor at the supplied voltage */
    public Command runVoltage(DoubleSupplier volts) {
        return Commands.run(
            () -> io.setVoltage(volts.getAsDouble()),
            this
        );
    }

    /** Set the motor to the specified velocity. Keeps running after the command ends! */
    public Command applyVelocity(double radPerSec) {
        return Commands.runOnce(
            () -> io.setVelocity(radPerSec),
            this
        );
    }

    /** Run the motor at the supplied velocity */
    public Command runVelocity(DoubleSupplier radPerSec) {
        return Commands.run(
            () -> io.setVelocity(radPerSec.getAsDouble()),
            this
        );
    }

    /** Feed pickleballs towards the drum while held, and stop when released */
    public Command index() {
        return startEnd(() -> io.setVoltage(IndexerConstants.kIndexVolts), () -> io.setVoltage(0));
    }

    /** Run the indexer in reverse while held (to clear jams), and stop when released */
    public Command reverse() {
        return startEnd(() -> io.setVoltage(IndexerConstants.kReverseVolts), () -> io.setVoltage(0));
    }

    /** Stop the motor */
    public Command stop() {
        return applyVoltage(0);
    }

    /** Gets the motor's voltage */
    public double getVoltage() {
        return inputs.appliedVoltage;
    }

    /** Gets the motor's velocity */
    public double getVelocityRadPerSec() {
        return inputs.velocityRadPerSec;
    }
}
