package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ElevatorSubsystem extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;

        this.sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
            null, Volts.of(1), null,
            (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())
            ),

            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                (log) ->
                    log.motor("Elevator Kraken")
                        .voltage(Volts.of(inputs.appliedVolts))
                        .linearPosition(Meters.of(inputs.positionMeters))
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

    public double getPosition() {
        return inputs.positionMeters;
    }

    public double getVelocity() {
        return inputs.velocityMetersPerSecond;
    }

    public double getStatorCurrent() {
        return inputs.statorCurrentAmps;
    }

    public Command setVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    public Command setPosition(double meters) {
        return Commands.runOnce(
            () -> io.setPosition(MathUtil.clamp(meters, ElevatorConstants.kBackwardPositionLimit, ElevatorConstants.kForwardPositionLimit)),
            this
        );
    }

    public Command stop() {
        return setVoltage(0);
    }

    public Command sysIdQuasistatic(Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysId.dynamic(direction);
    }

}