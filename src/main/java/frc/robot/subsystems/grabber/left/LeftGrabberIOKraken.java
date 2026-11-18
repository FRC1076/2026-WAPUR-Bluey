package frc.robot.subsystems.grabber.left;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.grabber.GrabberConstants;
import lib.units.TalonFXUnitConverter;

public class LeftGrabberIOKraken implements LeftGrabberIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;
    private final TalonFXUnitConverter m_unitConverter;

    private final VoltageOut m_voltageRequest;
    private final PositionVoltage m_positionRequest;
    private final VelocityVoltage m_velocityRequest;

    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<Angle> m_positionSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Temperature> m_temperatureSignal;

    public LeftGrabberIOKraken() {
        m_motor = new TalonFX(LeftGrabberConstants.kMotorCANID, GrabberConstants.kCANBus);

        m_motorConfig = new TalonFXConfiguration();

        m_unitConverter = new TalonFXUnitConverter();

        m_motorConfig.Voltage.PeakForwardVoltage = GrabberConstants.kPeakForwardVoltage;
        m_motorConfig.Voltage.PeakReverseVoltage = GrabberConstants.kPeakReverseVoltage;

        m_motorConfig.CurrentLimits.SupplyCurrentLimit = GrabberConstants.kSupplyCurrentLimit;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = GrabberConstants.kStatorCurrentLimit;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        //Inverted or not
        m_motorConfig.MotorOutput.Inverted = LeftGrabberConstants.kInverted;
        m_motorConfig.MotorOutput.NeutralMode = GrabberConstants.kNeutralMode;

        m_motorConfig.Feedback.SensorToMechanismRatio = LeftGrabberConstants.kGearRatio;
        m_motorConfig.Feedback.VelocityFilterTimeConstant = LeftGrabberConstants.kVelocityFilterTimeConstant;

        // Slot 0: velocity control
        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkP(LeftGrabberConstants.VelocityControl.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkI(LeftGrabberConstants.VelocityControl.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkD(LeftGrabberConstants.VelocityControl.kD);
        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkS(LeftGrabberConstants.VelocityControl.kS);
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkV(LeftGrabberConstants.VelocityControl.kV);
        m_motorConfig.Slot0.kA = m_unitConverter.fromSIkA(LeftGrabberConstants.VelocityControl.kA);

        // Slot 1: position control
        m_motorConfig.Slot1.kP = m_unitConverter.fromSIkP(LeftGrabberConstants.PositionControl.kP);
        m_motorConfig.Slot1.kI = m_unitConverter.fromSIkI(LeftGrabberConstants.PositionControl.kI);
        m_motorConfig.Slot1.kD = m_unitConverter.fromSIkD(LeftGrabberConstants.PositionControl.kD);
        m_motorConfig.Slot1.kS = m_unitConverter.fromSIkS(LeftGrabberConstants.PositionControl.kS);

        m_motor.getConfigurator().apply(m_motorConfig);

        m_voltageRequest = new VoltageOut(0).withEnableFOC(GrabberConstants.kUseFOC);
        m_velocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(GrabberConstants.kUseFOC);
        m_positionRequest = new PositionVoltage(0).withSlot(1).withEnableFOC(GrabberConstants.kUseFOC);

        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_positionSignal = m_motor.getPosition();
        m_velocitySignal = m_motor.getVelocity();
        m_temperatureSignal = m_motor.getDeviceTemp();
    }

    @Override
    public void updateInputs(LeftGrabberIOInputs inputs) {
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
    public void runVolts(double voltage) {
        m_motor.setControl(m_voltageRequest.withOutput(voltage));
    }

    @Override
    public void setPosition(double radians) {
        m_motor.setControl(m_positionRequest.withPosition(m_unitConverter.fromSIPos(radians)));
    }

    @Override
    public void setVelocity(double radPerSec) {
        if (radPerSec == 0) {
            // Coast/brake to a stop instead of having kS fight around zero
            runVolts(0);
        } else {
            m_motor.setControl(m_velocityRequest.withVelocity(m_unitConverter.fromSIVel(radPerSec)));
        }
    }
}
