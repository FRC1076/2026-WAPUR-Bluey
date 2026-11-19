package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ElevatorSubsystem extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    // The height the elevator was last told to go to
    private double m_targetMeters = ElevatorConstants.kBackwardPositionLimit;

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
        Logger.recordOutput("Elevator/TargetMeters", m_targetMeters);
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

    public Command applyVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    /** Tells the elevator to go to (and hold) the given height */
    public Command applyPosition(double meters) {
        return Commands.runOnce(
            () -> {
                m_targetMeters = MathUtil.clamp(meters, ElevatorConstants.kBackwardPositionLimit, ElevatorConstants.kForwardPositionLimit);
                io.setPosition(m_targetMeters);
            },
            this
        );
    }

    /** Sends the elevator to a scoring level (1-4) */
    public Command goToLevel(int level) {
        double height = switch (level) {
            case 1 -> ElevatorConstants.kLevel1HeightMeters;
            case 2 -> ElevatorConstants.kLevel2HeightMeters;
            case 3 -> ElevatorConstants.kLevel3HeightMeters;
            case 4 -> ElevatorConstants.kLevel4HeightMeters;
            default -> ElevatorConstants.kLevel1HeightMeters;
        };

        if (level > 4 || level < 1) {
            DriverStation.reportError("Tried to set the elevator to an invalid level. Applying level one instead.", true);
        }

        return applyPosition(height);
    }

    /** Moves the elevator target up by one increment */
    public Command incrementUp() {
        return Commands.defer(() -> applyPosition(m_targetMeters + ElevatorConstants.kIncrementMeters), Set.of(this));
    }

    /** Moves the elevator target down by one increment */
    public Command incrementDown() {
        return Commands.defer(() -> applyPosition(m_targetMeters - ElevatorConstants.kIncrementMeters), Set.of(this));
    }

    /** The height the elevator was last told to go to */
    public double getTargetMeters() {
        return m_targetMeters;
    }

    public Command stop() {
        return applyVoltage(0);
    }

    public Command sysIdQuasistatic(Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysId.dynamic(direction);
    }

}