package frc.robot.subsystems.pickleball_Intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.CAN;
import lib.units.TalonFXUnitConverter;

public class pickleball_IntakeIOKraken implements pickleball_IntakeIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;
    private final TalonFXUnitConverter m_unitConverter;
    private final VoltageOut m_voltageRequst;
    private final VelocityVoltage m_velocityRequest;


    public pickleball_IntakeIOKraken() {

        m_motor = new TalonFX(pickleball_IntakeConstants.kCANID); 
        m_motorConfig = new TalonFXConfiguration();
        m_unitConverter = new TalonFXUnitConverter();

        m_motorConfig.CurrentLimits.SupplyCurrentLimit = pickleball_IntakeConstants.kSupplyCurrentLimit;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = pickleball_IntakeConstants.kStatorCurrentLimit;

        m_motorConfig.MotorOutput.Inverted = pickleball_IntakeConstants.kInvertedValue;
        m_motorConfig.Feedback.SensorToMechanismRatio = pickleball_IntakeConstants.kGearRatio;

        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkP(pickleball_IntakeConstants.Control.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkI(pickleball_IntakeConstants.Control.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkD(pickleball_IntakeConstants.Control.kD);
        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkS(pickleball_IntakeConstants.Control.kS);
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkV(pickleball_IntakeConstants.Control.kV);
        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkA(pickleball_IntakeConstants.Control.kS);

        m_motor.getConfigurator().apply(m_motorConfig);

        m_voltageRequst = new VoltageOut(0).withEnableFOC(pickleball_IntakeConstants.kUseFOC);
        m_velocityRequest = new VelocityVoltage(null).withEnableFOC(false);

    }

    @Override
    public void updateInputs(pickleball_IntakeIOInputs inputs) {
        inputs.appliedVolts = m_motor.getMotorVoltage().getValue().in(Volts);
        inputs.postion = m_motor.getPosition().getValue().in(Radians);
        inputs.RPS = m_motor.getVelocity().getValue().in(RadiansPerSecond);
    }

    @Override
    public void runVolts(double volts) {
        m_motor.setVoltage(volts);
    }

//.withVelocity(RadiansPerSecond.of(radiansPerSecond))

    @Override
    public void setRPS(double RadsPerSec){
        m_motor.setControl(m_velocityRequest.withVelocity(RadiansPerSecond.of(RadiansPerSecond)));
    }

}
