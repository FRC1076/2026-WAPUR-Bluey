// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPositions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, Volts.of(2), null,
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                (log) ->
                    log.motor("Elevator Kraken")
                        .voltage(Volts.of(inputs.appliedVoltage))
                        .linearPosition(Meters.of(inputs.heightMeters))
                        .linearVelocity(MetersPerSecond.of(inputs.velocityMetersPerSecond)),
                this
            )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public double getHeightMeters() {
        return inputs.heightMeters;
    }

    public boolean withinTolerance(double targetMeters) {
        return Math.abs(getHeightMeters() - targetMeters) < ElevatorConstants.kHeightToleranceMeters;
    }

    public boolean withinTolerance(DoubleSupplier targetMeters) {
        return withinTolerance(targetMeters.getAsDouble());
    }

    /** Trigger that is true when the elevator is running closed-loop and has reached its target */
    public Trigger atSetpoint() {
        return new Trigger(() -> inputs.PIDActive && withinTolerance(inputs.PIDTargetMeters));
    }

    /** Set the voltage applied to the motor with software stops enabled */
    public Command applyVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    /** Run the motor at the supplied voltage with software stops enabled */
    public Command runVoltage(DoubleSupplier volts) {
        return Commands.run(
            () -> io.setVoltage(volts.getAsDouble()),
            this
        );
    }

    /** Set the voltage applied to the motor with software stops disabled */
    public Command applyVoltageUnrestricted(double volts) {
        return Commands.runOnce(
            () -> io.setVoltageNoSoftStops(volts),
            this
        );
    }

    /** Run the motor at the supplied voltage with software stops disabled */
    public Command runVoltageUnrestricted(DoubleSupplier volts) {
        return Commands.run(
            () -> io.setVoltageNoSoftStops(volts.getAsDouble()),
            this
        );
    }

    /** Tell the elevator to go to the specified height */
    public Command applyPosition(double meters) {
        return Commands.runOnce(
            () -> io.setPosition(MathUtil.clamp(meters, ElevatorConstants.kMinHeightMeters, ElevatorConstants.kMaxHeightMeters)),
            this
        );
    }

    /** Tell the elevator to go to the specified preset height */
    public Command applyPosition(ElevatorPositions position) {
        return applyPosition(position.heightMeters);
    }

    /** Run the elevator to the supplied height */
    public Command runPosition(DoubleSupplier meters) {
        return Commands.run(
            () -> io.setPosition(MathUtil.clamp(meters.getAsDouble(), ElevatorConstants.kMinHeightMeters, ElevatorConstants.kMaxHeightMeters)),
            this
        );
    }

    /** Tell the elevator to go to the specified preset height, and wait until it gets there */
    public Command applyPositionAndWait(ElevatorPositions position) {
        return applyPosition(position)
            .andThen(Commands.waitUntil(() -> withinTolerance(position.heightMeters)));
    }

    /** Hold the elevator at its current height */
    public Command holdPosition() {
        return Commands.defer(
            () -> applyPosition(getHeightMeters()),
            Set.of(this)
        );
    }

    /** Manually control the elevator with a joystick-style supplier in the range [-1, 1] */
    public Command runOperatorControl(DoubleSupplier speed) {
        return runVoltage(() -> MathUtil.clamp(speed.getAsDouble(), -1, 1) * ElevatorConstants.kMaxOperatorControlVolts);
    }

    /** Set the current position of the elevator to zero */
    public Command rezeroElevator() {
        return Commands.runOnce(() -> io.rezero()).ignoringDisable(true);
    }

    public Command sysIdQuasistatic(Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysId.dynamic(direction);
    }
}
