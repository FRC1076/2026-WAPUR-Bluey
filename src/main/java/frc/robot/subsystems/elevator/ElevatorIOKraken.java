package frc.robot.subsystems.elevator;

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

public class ElevatorIOKraken implements ElevatorIO{

    // creation
    private final TalonFX m_motor = new TalonFX(ElevatorConstants.kLeaderID); // TODO: add canbus 

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


        // feedforward
        m_config.Slot0.kS = ElevatorConstants.kS;
        m_config.Slot0.kV = ElevatorConstants.kV; 
        m_config.Slot0.kA = ElevatorConstants.kA;
        m_config.Slot0.kG = ElevatorConstants.kG;
        m_config.Slot0.kP = ElevatorConstants.kP;
        m_config.Slot0.kI = ElevatorConstants.kI;
        m_config.Slot0.kD = ElevatorConstants.kD;
        m_config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // voltage and current limits
        m_config.Voltage.PeakForwardVoltage = ElevatorConstants.kPeakForwardVoltage;
        m_config.Voltage.PeakReverseVoltage = -ElevatorConstants.kPeakReverseVoltage;
        m_config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.kSupplyCurrentLimit;
        m_config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kStatorCurrentLimit;
        m_config.CurrentLimits.SupplyCurrentLimitEnable = true;


        // soft limits
        m_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.kForwardPositionLimit;
        m_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.kBackwardPositionLimit;

        // status signal setup

        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_statorCurrentSignal = m_motor.getStatorCurrent();
        m_temperatureSignal = m_motor.getDeviceTemp();
        m_velocitySignal = m_motor.getVelocity();
        m_positionSignal = m_motor.getPosition();
    
        // set motion magic settings
        var motionMagicConfigs = m_config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MotionMagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = ElevatorConstants.MotionMagicJerk; 

        m_motor.getConfigurator().apply(m_config);
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
        inputs.velocityMetersPerSecond = rotationsToMeters(m_velocitySignal.getValueAsDouble());
        inputs.positionMeters = rotationsToMeters(m_positionSignal.getValueAsDouble());

    }

    @Override
    public void setPosition(double heightMeters){
        m_positionRequest.withPosition(heightMeters);
        m_motor.setControl(m_positionRequest);
    }

    @Override
    public void setVoltage(double volts){
        // really only used for stop
        m_voltageRequest.withOutput(volts);
        m_motor.setControl(m_voltageRequest);
    }

    @Override
    public void stop(){
        setVoltage(0);
    }
    
    private static double rotationsToMeters(double rotations) { 
        return rotations * ElevatorConstants.kMetersPerDrumRotation; 
    }

    private static double metersToRotations(double meters) { 
        return meters  / ElevatorConstants.kMetersPerDrumRotation;
    }

}