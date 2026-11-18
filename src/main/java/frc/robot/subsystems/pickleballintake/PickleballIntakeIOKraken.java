package frc.robot.subsystems.pickleballintake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import lib.units.TalonFXUnitConverter;

public class PickleballIntakeIOKraken implements PickleballIntakeIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;
    private final TalonFXUnitConverter m_unitConverter;
    private final VoltageOut m_voltageRequest;
    private final VelocityVoltage m_velocityRequest;

    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<Angle> m_positionSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Temperature> m_temperatureSignal;

    public PickleballIntakeIOKraken() {

        m_motor = new TalonFX(PickleballIntakeConstants.kCANID, PickleballIntakeConstants.kCanBus);
        m_motorConfig = new TalonFXConfiguration();
        m_unitConverter = new TalonFXUnitConverter();

        m_motorConfig.Voltage.PeakForwardVoltage = PickleballIntakeConstants.kPeakForwardVoltage;
        m_motorConfig.Voltage.PeakReverseVoltage = PickleballIntakeConstants.kPeakReverseVoltage;

        m_motorConfig.CurrentLimits.SupplyCurrentLimit = PickleballIntakeConstants.kSupplyCurrentLimit;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = PickleballIntakeConstants.kStatorCurrentLimit;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        m_motorConfig.MotorOutput.Inverted = PickleballIntakeConstants.kInvertedValue;
        m_motorConfig.MotorOutput.NeutralMode = PickleballIntakeConstants.kNeutralMode;
        m_motorConfig.Feedback.SensorToMechanismRatio = PickleballIntakeConstants.kGearRatio;

        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkP(PickleballIntakeConstants.Control.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkI(PickleballIntakeConstants.Control.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkD(PickleballIntakeConstants.Control.kD);
        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkS(PickleballIntakeConstants.Control.kS);
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkV(PickleballIntakeConstants.Control.kV);
        m_motorConfig.Slot0.kA = m_unitConverter.fromSIkA(PickleballIntakeConstants.Control.kA);

        m_motor.getConfigurator().apply(m_motorConfig);

        m_voltageRequest = new VoltageOut(0).withEnableFOC(PickleballIntakeConstants.kUseFOC);
        m_velocityRequest = new VelocityVoltage(0).withEnableFOC(PickleballIntakeConstants.kUseFOC);

        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_positionSignal = m_motor.getPosition();
        m_velocitySignal = m_motor.getVelocity();
        m_temperatureSignal = m_motor.getDeviceTemp();
    }

    @Override
    public void updateInputs(PickleballIntakeIOInputs inputs) {
        StatusSignal.refreshAll(
            m_voltageSignal,
            m_currentSignal,
            m_positionSignal,
            m_velocitySignal,
            m_temperatureSignal
        );

        inputs.appliedVolts = m_voltageSignal.getValue().in(Volts);
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
        inputs.positionRadians = m_unitConverter.toSIPos(m_positionSignal.getValueAsDouble());
        inputs.velocityRadPerSec = m_unitConverter.toSIVel(m_velocitySignal.getValueAsDouble());
        inputs.motorTempDegC = m_temperatureSignal.getValueAsDouble();
    }

    @Override
    public void runVolts(double volts) {
        m_motor.setControl(m_voltageRequest.withOutput(volts));
    }

    @Override
    public void setVelocity(double radPerSec) {
        if (radPerSec == 0) {
            runVolts(0);
        } else {
            m_motor.setControl(m_velocityRequest.withVelocity(m_unitConverter.fromSIVel(radPerSec)));
        }
    }
}
