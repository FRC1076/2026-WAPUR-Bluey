// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.drum;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class DrumSubsystem extends SubsystemBase {
    private final DrumIO io;
    private final DrumIOInputsAutoLogged inputs = new DrumIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public DrumSubsystem(DrumIO io) {
        this.io = io;

        // Set up SysId
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null,
                (state) -> Logger.recordOutput("Drum/SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                (log) ->
                    log.motor("Drum Kraken")
                        .voltage(Volts.of(inputs.appliedVoltage))
                        .angularVelocity(RadiansPerSecond.of(inputs.velocityRadPerSec)),
                this
            )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drum", inputs);
    }

    /** Returns whether the drum is within tolerance of the specified velocity */
    public boolean readyToShoot(double targetVelocityRadPerSec) {
        return Math.abs(targetVelocityRadPerSec - inputs.velocityRadPerSec) < DrumConstants.kSetpointToleranceRadPerSec;
    }

    /** Returns whether the drum is running closed-loop and within tolerance of its current target */
    public boolean atSetpoint() {
        return inputs.PIDActive && readyToShoot(inputs.velocityPIDTargetRadPerSec);
    }

    /** Trigger that is true when the drum is up to speed */
    public Trigger isReadyToShoot() {
        return new Trigger(this::atSetpoint);
    }

    /** Set the voltage of the motor */
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

    /** Tell the drum to run at the specified velocity */
    public Command applyVelocity(double radPerSec) {
        return Commands.runOnce(
            () -> io.setVelocity(radPerSec),
            this
        );
    }

    /** Run the drum at the supplied velocity */
    public Command runVelocity(DoubleSupplier radPerSec) {
        return Commands.run(
            () -> io.setVelocity(radPerSec.getAsDouble()),
            this
        );
    }

    /** Spin the drum up to the default shooting velocity */
    public Command spinUp() {
        return applyVelocity(DrumConstants.kShootVelocityRadPerSec);
    }

    /** Spin the drum up and wait until it reaches the default shooting velocity */
    public Command spinUpAndWait() {
        return spinUp().andThen(Commands.waitUntil(this::atSetpoint));
    }

    /** Increase the drum's target velocity by one increment */
    public Command incrementVelocity() {
        return Commands.defer(
            () -> applyVelocity(MathUtil.clamp(
                getPIDTargetRadPerSec() + DrumConstants.kVelocityIncrementRadPerSec,
                0, DrumConstants.kMaxVelocityRadPerSec)),
            Set.of(this)
        );
    }

    /** Decrease the drum's target velocity by one increment */
    public Command decrementVelocity() {
        return Commands.defer(
            () -> applyVelocity(MathUtil.clamp(
                getPIDTargetRadPerSec() - DrumConstants.kVelocityIncrementRadPerSec,
                0, DrumConstants.kMaxVelocityRadPerSec)),
            Set.of(this)
        );
    }

    /** Run the drum backwards to clear jams */
    public Command reverse() {
        return applyVoltage(DrumConstants.kReverseVolts);
    }

    /** Stop the drum */
    public Command stop() {
        return applyVoltage(0);
    }

    // Make SysId quasistatic and dynamic commands
    public Command sysIdQuasistatic(Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysId.dynamic(direction);
    }

    // Getters for data
    public double getVelocityRadPerSec() {
        return inputs.velocityRadPerSec;
    }

    public double getPIDTargetRadPerSec() {
        return inputs.velocityPIDTargetRadPerSec;
    }
}
