package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import lib.units.TalonFXUnitConverter;

public class ElevatorIOKraken implements ElevatorIO{

    // creation
    private final TalonFX m_motor = new TalonFX(ElevatorConstants.kLeaderID, ElevatorConstants.kCANBus);

    // converts between drum rotations (what the TalonFX uses) and meters of carriage travel
    private final TalonFXUnitConverter m_unitConverter = new TalonFXUnitConverter(ElevatorConstants.kMetersPerDrumRotation);

    // magic motion
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut m_voltageRequest = new VoltageOut(0);

    // status signals
    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<Current> m_statorCurrentSignal;
    private final StatusSignal<Temperature> m_temperatureSignal;
    private final StatusSignal<Angle> m_positionSignal;

    public ElevatorIOKraken() {
        TalonFXConfiguration m_config = new TalonFXConfiguration();

        m_config.Feedback.SensorToMechanismRatio = ElevatorConstants.kSensorToMechanismRatio;
        m_config.MotorOutput.Inverted = ElevatorConstants.kInverted;
        m_config.MotorOutput.NeutralMode = ElevatorConstants.kNeutralMode;


        // feedforward (kS and kG are plain volts, so they don't get converted)
        m_config.Slot0.kS = m_unitConverter.fromSIkS(ElevatorConstants.kS);
        m_config.Slot0.kV = m_unitConverter.fromSIkV(ElevatorConstants.kV);
        m_config.Slot0.kA = m_unitConverter.fromSIkA(ElevatorConstants.kA);
        m_config.Slot0.kG = ElevatorConstants.kG;
        m_config.Slot0.kP = m_unitConverter.fromSIkP(ElevatorConstants.kP);
        m_config.Slot0.kI = m_unitConverter.fromSIkI(ElevatorConstants.kI);
        m_config.Slot0.kD = m_unitConverter.fromSIkD(ElevatorConstants.kD);
        m_config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // voltage and current limits
        m_config.Voltage.PeakForwardVoltage = ElevatorConstants.kPeakForwardVoltage;
        m_config.Voltage.PeakReverseVoltage = ElevatorConstants.kPeakReverseVoltage;
        m_config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.kSupplyCurrentLimit;
        m_config.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kStatorCurrentLimit;
        m_config.CurrentLimits.StatorCurrentLimitEnable = true;


        // soft limits (TalonFX wants drum rotations)
        m_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = m_unitConverter.fromSIPos(ElevatorConstants.kForwardPositionLimit);
        m_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = m_unitConverter.fromSIPos(ElevatorConstants.kBackwardPositionLimit);

        // set motion magic settings
        var motionMagicConfigs = m_config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = m_unitConverter.fromSIVel(ElevatorConstants.kCruiseVelocityMetersPerSec);
        motionMagicConfigs.MotionMagicAcceleration = m_unitConverter.fromSIAccel(ElevatorConstants.kAccelerationMetersPerSec2);
        motionMagicConfigs.MotionMagicJerk = m_unitConverter.fromSIJerk(ElevatorConstants.kJerkMetersPerSec3);

        m_motor.getConfigurator().apply(m_config);

        // status signal setup
        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_statorCurrentSignal = m_motor.getStatorCurrent();
        m_temperatureSignal = m_motor.getDeviceTemp();
        m_velocitySignal = m_motor.getVelocity();
        m_positionSignal = m_motor.getPosition();

        // optimizeBusUtilization turns off every signal that doesn't have a frequency set,
        // so the ones we read need a frequency first
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            m_voltageSignal,
            m_currentSignal,
            m_statorCurrentSignal,
            m_temperatureSignal,
            m_velocitySignal,
            m_positionSignal
        );
        m_motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        StatusSignal.refreshAll(
            m_voltageSignal,
            m_currentSignal,
            m_statorCurrentSignal,
            m_temperatureSignal,
            m_velocitySignal,
            m_positionSignal
        );

        inputs.appliedVolts = m_voltageSignal.getValueAsDouble();
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
        inputs.statorCurrentAmps = m_statorCurrentSignal.getValueAsDouble();
        inputs.motorTempDegreesC = m_temperatureSignal.getValueAsDouble();
        inputs.velocityMetersPerSecond = m_unitConverter.toSIVel(m_velocitySignal.getValueAsDouble());
        inputs.positionMeters = m_unitConverter.toSIPos(m_positionSignal.getValueAsDouble());

    }

    @Override
    public void setPosition(double heightMeters){
        m_positionRequest.withPosition(m_unitConverter.fromSIPos(heightMeters));
        m_motor.setControl(m_positionRequest);
    }

    @Override
    public void setVoltage(double volts){
        m_voltageRequest.withOutput(volts);
        m_motor.setControl(m_voltageRequest);
    }

    @Override
    public void stop(){
        setVoltage(0);
    }
}
