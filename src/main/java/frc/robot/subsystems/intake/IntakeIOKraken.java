// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.intake;

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

public class IntakeIOKraken implements IntakeIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    private final TalonFXUnitConverter m_unitConverter = new TalonFXUnitConverter();

    private final VoltageOut m_voltageRequest;
    private final VelocityTorqueCurrentFOC m_velocityRequest;

    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Temperature> m_temperatureSignal;

    public IntakeIOKraken() {
        m_motor = new TalonFX(IntakeConstants.kMotorCANId, IntakeConstants.kCANBus);

        m_motorConfig = new TalonFXConfiguration();

        // Configure voltage and current limits
        m_motorConfig.Voltage.PeakForwardVoltage = IntakeConstants.kMaxVoltage;
        m_motorConfig.Voltage.PeakReverseVoltage = -IntakeConstants.kMaxVoltage;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kSupplyCurrentLimit;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.kStatorCurrentLimit;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        m_motorConfig.MotorOutput.Inverted = IntakeConstants.kInverted;
        m_motorConfig.MotorOutput.NeutralMode = IntakeConstants.kNeutralMode;

        m_motorConfig.Feedback.SensorToMechanismRatio = IntakeConstants.kGearRatio;
        m_motorConfig.Feedback.VelocityFilterTimeConstant = IntakeConstants.kVelocityFilterTimeConstant;

        // Configure closed-loop gains
        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkP(IntakeConstants.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkI(IntakeConstants.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkD(IntakeConstants.kD);
        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkS(IntakeConstants.kS);
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkV(IntakeConstants.kV);
        m_motorConfig.Slot0.kA = m_unitConverter.fromSIkA(IntakeConstants.kA);

        m_motor.getConfigurator().apply(m_motorConfig);

        m_voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.kUseFOC);
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
    public void updateInputs(IntakeIOInputs inputs) {
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
