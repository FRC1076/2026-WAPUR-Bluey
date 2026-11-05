// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import lib.units.TalonFXUnitConverter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOKraken implements ElevatorIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    private final VoltageOut m_voltageRequest;
    private final VoltageOut m_voltageNoSoftStopsRequest;
    private final MotionMagicVoltage m_positionRequest;

    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Temperature> m_temperatureSignal;
    private final StatusSignal<Angle> m_positionSignal;

    // Converts output shaft rotations to meters of carriage travel
    private final TalonFXUnitConverter m_unitConverter;

    public ElevatorIOKraken() {
        m_motor = new TalonFX(ElevatorConstants.kMotorCANId, ElevatorConstants.kCANBus);

        m_motorConfig = new TalonFXConfiguration();
        m_unitConverter = new TalonFXUnitConverter(ElevatorConstants.kMetersPerRotation);

        // Configure voltage and current limits
        m_motorConfig.Voltage.PeakForwardVoltage = ElevatorConstants.kMaxVoltage;
        m_motorConfig.Voltage.PeakReverseVoltage = -ElevatorConstants.kMaxVoltage;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.kSupplyCurrentLimit;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kStatorCurrentLimit;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Gearing only; meters conversion is handled by the unit converter
        m_motorConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.kGearRatio;

        m_motorConfig.MotorOutput.Inverted = ElevatorConstants.kInverted;
        m_motorConfig.MotorOutput.NeutralMode = ElevatorConstants.kNeutralMode;

        // Software limits
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = m_unitConverter.fromSIPos(ElevatorConstants.kMaxHeightMeters);
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = m_unitConverter.fromSIPos(ElevatorConstants.kMinHeightMeters);
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // PID and feedforward
        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkP(ElevatorConstants.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkI(ElevatorConstants.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkD(ElevatorConstants.kD);

        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkS(ElevatorConstants.kS);
        m_motorConfig.Slot0.kG = ElevatorConstants.kG; // Elevator_Static kG is a constant voltage, no conversion needed
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkV(ElevatorConstants.kV);
        m_motorConfig.Slot0.kA = m_unitConverter.fromSIkA(ElevatorConstants.kA);
        m_motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Motion Magic
        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity = m_unitConverter.fromSIVel(ElevatorConstants.kCruiseVelocityMetersPerSec);
        m_motorConfig.MotionMagic.MotionMagicAcceleration = m_unitConverter.fromSIAccel(ElevatorConstants.kMaxAccelMetersPerSec2);
        m_motorConfig.MotionMagic.MotionMagicJerk = m_unitConverter.fromSIJerk(ElevatorConstants.kMaxJerkMetersPerSec3);

        m_motor.getConfigurator().apply(m_motorConfig);

        // Assume the elevator starts at the bottom
        m_motor.setPosition(0);

        m_voltageRequest = new VoltageOut(0)
            .withEnableFOC(ElevatorConstants.kUseFOC);
        m_voltageNoSoftStopsRequest = new VoltageOut(0)
            .withEnableFOC(ElevatorConstants.kUseFOC)
            .withIgnoreSoftwareLimits(true);
        m_positionRequest = new MotionMagicVoltage(0)
            .withEnableFOC(ElevatorConstants.kUseFOC);

        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_velocitySignal = m_motor.getVelocity();
        m_temperatureSignal = m_motor.getDeviceTemp();
        m_positionSignal = m_motor.getPosition();
    }

    @Override
    public void setVoltage(double volts) {
        m_voltageRequest.withOutput(volts);
        m_motor.setControl(m_voltageRequest);
    }

    @Override
    public void setVoltageNoSoftStops(double volts) {
        m_voltageNoSoftStopsRequest.withOutput(volts);
        m_motor.setControl(m_voltageNoSoftStopsRequest);
    }

    @Override
    public void setPosition(double meters) {
        m_positionRequest.withPosition(m_unitConverter.fromSIPos(meters));
        m_motor.setControl(m_positionRequest);
    }

    @Override
    public void rezero() {
        m_motor.setPosition(0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        StatusSignal.refreshAll(
            m_voltageSignal,
            m_currentSignal,
            m_temperatureSignal,
            m_velocitySignal,
            m_positionSignal
        );

        inputs.appliedVoltage = m_voltageSignal.getValue().in(Volts);
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
        inputs.heightMeters = m_unitConverter.toSIPos(m_positionSignal.getValueAsDouble());
        inputs.velocityMetersPerSecond = m_unitConverter.toSIVel(m_velocitySignal.getValueAsDouble());
        inputs.motorTempDegC = m_temperatureSignal.getValueAsDouble();

        if (m_motor.getAppliedControl() instanceof MotionMagicVoltage mmv) {
            inputs.PIDActive = true;
            inputs.PIDTargetMeters = m_unitConverter.toSIPos(mmv.Position);
        } else {
            inputs.PIDActive = false;
            inputs.PIDTargetMeters = 0;
        }
    }
}
