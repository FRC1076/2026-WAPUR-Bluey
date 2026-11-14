package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import lib.units.TalonFXUnitConverter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;

public class GrabberIOKraken implements GrabberIO {
    private final TalonFX m_leftMotor;
    private final TalonFX m_rightMotor;
    private final TalonFXConfiguration m_rightMotorConfig;
    private final TalonFXConfiguration m_leftMotorConfig;
    private final TalonFXUnitConverter m_unitConverter;
    private final PositionVoltage m_leftPositionRequest = new PositionVoltage(0).withEnableFOC(false);
    private final PositionVoltage m_rightPositionRequest = new PositionVoltage(0).withEnableFOC(false);
    private final VelocityVoltage m_rightVelocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final VelocityVoltage m_leftVelocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final VoltageOut m_rightVoltageRequest;
    private final VoltageOut m_leftVoltageRequest;
    private final VelocityTorqueCurrentFOC m_VelocityTorqueCurrentFOC;

    public GrabberIOKraken() {

        m_rightMotor = new TalonFX(GrabberConstants.kRightMotorCANID);
        m_leftMotor = new TalonFX(GrabberConstants.kLeftMotorCANID);

        m_rightMotorConfig = new TalonFXConfiguration();
        m_leftMotorConfig = new TalonFXConfiguration();

        m_unitConverter = new TalonFXUnitConverter();

        m_rightMotorConfig.Voltage.PeakForwardVoltage = GrabberConstants.kPeakForwardVoltage;
        m_leftMotorConfig.Voltage.PeakForwardVoltage = GrabberConstants.kPeakForwardVoltage;
        m_rightMotorConfig.Voltage.PeakReverseVoltage = GrabberConstants.kPeakReverseVoltage;
        m_leftMotorConfig.Voltage.PeakReverseVoltage = GrabberConstants.kPeakReverseVoltage;

        m_rightMotorConfig.CurrentLimits.SupplyCurrentLimit = GrabberConstants.kSupplyCurrentLimit;
        m_leftMotorConfig.CurrentLimits.SupplyCurrentLimit = GrabberConstants.kSupplyCurrentLimit;

        m_rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_leftMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        m_rightMotorConfig.CurrentLimits.StatorCurrentLimit = GrabberConstants.kStatorCurrentLimit;
        m_leftMotorConfig.CurrentLimits.StatorCurrentLimit = GrabberConstants.kStatorCurrentLimit;

        //Inverted or not 
        m_rightMotorConfig.MotorOutput.Inverted = GrabberConstants.kRightInverted;
        m_leftMotorConfig.MotorOutput.Inverted = GrabberConstants.kLeftInverted;

        m_rightMotorConfig.MotorOutput.NeutralMode = GrabberConstants.kNeutralMode;
        m_leftMotorConfig.MotorOutput.NeutralMode = GrabberConstants.kNeutralMode;
        m_rightMotorConfig.Feedback.SensorToMechanismRatio = GrabberConstants.kRightGearRatio;
        m_leftMotorConfig.Feedback.SensorToMechanismRatio =GrabberConstants.kLeftGearRatio;

        m_rightMotorConfig.Slot0.kP = m_unitConverter.fromSIkP(GrabberConstants.RightControl.kP);
        m_leftMotorConfig.Slot0.kP = m_unitConverter.fromSIkP(GrabberConstants.LeftControl.kP);
        m_rightMotorConfig.Slot0.kI = m_unitConverter.fromSIkI(GrabberConstants.RightControl.kI);
        m_leftMotorConfig.Slot0.kI = m_unitConverter.fromSIkI(GrabberConstants.LeftControl.kI);
        m_rightMotorConfig.Slot0.kD = m_unitConverter.fromSIkD(GrabberConstants.RightControl.kD);
        m_leftMotorConfig.Slot0.kD = m_unitConverter.fromSIkD(GrabberConstants.LeftControl.kD);
        m_rightMotorConfig.Slot0.kS = m_unitConverter.fromSIkS(GrabberConstants.RightControl.kS);
        m_leftMotorConfig.Slot0.kS = m_unitConverter.fromSIkS(GrabberConstants.LeftControl.kS);
        m_rightMotorConfig.Slot0.kV = m_unitConverter.fromSIkV(GrabberConstants.RightControl.kV);
        m_leftMotorConfig.Slot0.kV = m_unitConverter.fromSIkV(GrabberConstants.LeftControl.kV);
        m_rightMotorConfig.Slot0.kA = m_unitConverter.fromSIkA(GrabberConstants.RightControl.kA);
        m_leftMotorConfig.Slot0.kA = m_unitConverter.fromSIkA(GrabberConstants.LeftControl.kA);


        m_rightMotor.getConfigurator().apply(m_rightMotorConfig);
        m_leftMotor.getConfigurator().apply(m_leftMotorConfig);

        m_rightVoltageRequest = new VoltageOut(0).withEnableFOC(GrabberConstants.kUseFOC);
        m_leftVoltageRequest = new VoltageOut(0).withEnableFOC(GrabberConstants.kUseFOC);
        
        

    

    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.leftAppliedVolts = m_leftMotor.getMotorVoltage().getValue().in(Volts);
        inputs.rightAppliedVolts = m_rightMotor.getMotorVoltage().getValue().in(Volts);
        inputs.leftPostion = m_leftMotor.getPosition().getValue().in(Radians);
        inputs.rightPostion = m_rightMotor.getPosition().getValue().in(Radians);
        inputs.leftRPS = m_leftMotor.getVelocity().getValue().in(RadiansPerSecond);
        inputs.rightRPS = m_rightMotor.getVelocity().getValue().in(RadiansPerSecond);
    }

    @Override
    public void leftRunVolts(double voltage) {
        m_leftMotor.setVoltage(voltage);
    }

    @Override
    public void rightRunVolts(double voltage) {
        m_rightMotor.setVoltage(voltage);
    }

    @Override
    public void leftSetPostion(double radians) {
        m_leftMotor.setControl(m_leftPositionRequest.withPosition(Radians.of(radians)));
    }

    @Override
    public void rightSetPostion(double radians) {
        m_rightMotor.setControl(m_rightPositionRequest.withPosition(Radians.of(radians)));
    }

    @Override
    public void leftSetRPS(double radiansPerSecond) {
        m_leftMotor.setControl(m_leftVelocityRequest.withVelocity(RadiansPerSecond.of(radiansPerSecond)));
    }

    @Override
    public void rightSetRPS(double radiansPerSecond) {
        m_rightMotor.setControl(m_rightVelocityRequest.withVelocity(RadiansPerSecond.of(radiansPerSecond)));
    }
}
