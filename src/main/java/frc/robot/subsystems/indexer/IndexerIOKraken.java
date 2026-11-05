// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import lib.units.TalonFXUnitConverter;

public class IndexerIOKraken implements IndexerIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    private final TalonFXUnitConverter m_unitConverter = new TalonFXUnitConverter();

    private final VoltageOut m_voltageRequest;
    private final VelocityTorqueCurrentFOC m_velocityRequest;

    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Temperature> m_temperatureSignal;

    public IndexerIOKraken() {
        m_motor = new TalonFX(IndexerConstants.kMotorCANId, IndexerConstants.kCANBus);

        m_motorConfig = new TalonFXConfiguration();

        // Configure voltage and current limits
        m_motorConfig.Voltage.PeakForwardVoltage = IndexerConstants.kMaxVoltage;
        m_motorConfig.Voltage.PeakReverseVoltage = -IndexerConstants.kMaxVoltage;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.kSupplyCurrentLimit;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.kStatorCurrentLimit;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        m_motorConfig.MotorOutput.Inverted = IndexerConstants.kInverted;
        m_motorConfig.MotorOutput.NeutralMode = IndexerConstants.kNeutralMode;

        m_motorConfig.Feedback.SensorToMechanismRatio = IndexerConstants.kGearRatio;
        m_motorConfig.Feedback.VelocityFilterTimeConstant = IndexerConstants.kVelocityFilterTimeConstant;

        // Configure closed-loop gains
        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkP(IndexerConstants.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkI(IndexerConstants.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkD(IndexerConstants.kD);
        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkS(IndexerConstants.kS);
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkV(IndexerConstants.kV);
        m_motorConfig.Slot0.kA = m_unitConverter.fromSIkA(IndexerConstants.kA);

        m_motor.getConfigurator().apply(m_motorConfig);

        m_voltageRequest = new VoltageOut(0).withEnableFOC(IndexerConstants.kUseFOC);
        m_velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_velocitySignal = m_motor.getVelocity();
        m_temperatureSignal = m_motor.getDeviceTemp();
    }

    /** Sets the voltage of the motor */
    @Override
    public void setVoltage(double volts) {
        m_voltageRequest.Output = volts;
        m_motor.setControl(m_voltageRequest);
    }

    /** Sets the velocity of the motor by FOC PID */
    @Override
    public void setVelocity(double radPerSec) {
        if (radPerSec == 0) {
            m_motor.setVoltage(0);
        } else {
            m_velocityRequest.Velocity = m_unitConverter.fromSIVel(radPerSec);
            m_motor.setControl(m_velocityRequest);
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        StatusSignal.refreshAll(
            m_voltageSignal,
            m_currentSignal,
            m_velocitySignal,
            m_temperatureSignal
        );

        inputs.appliedVoltage = m_voltageSignal.getValue().in(Volts);
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
        inputs.velocityRadPerSec = m_unitConverter.toSIVel(m_velocitySignal.getValueAsDouble());
        inputs.motorTempDegC = m_temperatureSignal.getValueAsDouble();

        if (m_motor.getAppliedControl() instanceof VelocityTorqueCurrentFOC vtcf) {
            inputs.PIDActive = true;
            inputs.velocityPIDTargetRadPerSec = m_unitConverter.toSIVel(vtcf.Velocity);
        } else {
            inputs.PIDActive = false;
            inputs.velocityPIDTargetRadPerSec = 0;
        }
    }
}
