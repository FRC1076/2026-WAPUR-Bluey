// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.drum;

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

public class DrumIOKraken implements DrumIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    private final TalonFXUnitConverter m_unitConverter = new TalonFXUnitConverter();

    // Control requests
    private final VoltageOut m_voltageRequest;
    private final VelocityTorqueCurrentFOC m_velocityRequest;

    // Status signals
    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Temperature> m_temperatureSignal;

    public DrumIOKraken() {
        m_motor = new TalonFX(DrumConstants.kMotorCANId, DrumConstants.kCANBus);

        m_motorConfig = new TalonFXConfiguration();

        // Configure voltage and current limits
        m_motorConfig.Voltage.PeakForwardVoltage = DrumConstants.kMaxVoltage;
        m_motorConfig.Voltage.PeakReverseVoltage = -DrumConstants.kMaxVoltage;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = DrumConstants.kSupplyCurrentLimit;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = DrumConstants.kStatorCurrentLimit;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        m_motorConfig.MotorOutput.Inverted = DrumConstants.kInverted;
        m_motorConfig.MotorOutput.NeutralMode = DrumConstants.kNeutralMode;

        m_motorConfig.Feedback.SensorToMechanismRatio = DrumConstants.kGearRatio;
        m_motorConfig.Feedback.VelocityFilterTimeConstant = DrumConstants.kVelocityFilterTimeConstant;

        // Configure closed-loop gains
        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkP(DrumConstants.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkI(DrumConstants.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkD(DrumConstants.kD);
        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkS(DrumConstants.kS);
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkV(DrumConstants.kV);
        m_motorConfig.Slot0.kA = m_unitConverter.fromSIkA(DrumConstants.kA);

        m_motor.getConfigurator().apply(m_motorConfig);

        m_voltageRequest = new VoltageOut(0)
            .withEnableFOC(DrumConstants.kUseFOC)
            .withOverrideBrakeDurNeutral(true);
        m_velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_velocitySignal = m_motor.getVelocity();
        m_temperatureSignal = m_motor.getDeviceTemp();
    }

    @Override
    public void setVoltage(double volts) {
        m_voltageRequest.withOutput(volts);
        m_motor.setControl(m_voltageRequest);
    }

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
    public void updateInputs(DrumIOInputs inputs) {
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
