package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class GrabberIOKraken implements GrabberIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final PositionVoltage leftPositionRequest = new PositionVoltage(0).withEnableFOC(false);
    private final PositionVoltage rightPositionRequest = new PositionVoltage(0).withEnableFOC(false);
    private final VelocityVoltage leftVelocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final VelocityVoltage rightVelocityRequest = new VelocityVoltage(0).withEnableFOC(false);

    public GrabberIOKraken(TalonFX leftMotor, TalonFX rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.leftAppliedVolts = leftMotor.getMotorVoltage().getValue().in(Volts);
        inputs.rightAppliedVolts = rightMotor.getMotorVoltage().getValue().in(Volts);
        inputs.leftPostion = leftMotor.getPosition().getValue().in(Radians);
        inputs.rightPostion = rightMotor.getPosition().getValue().in(Radians);
        inputs.leftRPS = leftMotor.getVelocity().getValue().in(RadiansPerSecond);
        inputs.rightRPS = rightMotor.getVelocity().getValue().in(RadiansPerSecond);
    }

    @Override
    public void leftRunVolts(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    @Override
    public void rightRunVolts(double voltage) {
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void leftSetPostion(double radians) {
        leftMotor.setControl(leftPositionRequest.withPosition(Radians.of(radians)));
    }

    @Override
    public void rightSetPostion(double radians) {
        rightMotor.setControl(rightPositionRequest.withPosition(Radians.of(radians)));
    }

    @Override
    public void leftSetRPS(double radiansPerSecond) {
        leftMotor.setControl(leftVelocityRequest.withVelocity(RadiansPerSecond.of(radiansPerSecond)));
    }

    @Override
    public void rightSetRPS(double radiansPerSecond) {
        rightMotor.setControl(rightVelocityRequest.withVelocity(RadiansPerSecond.of(radiansPerSecond)));
    }
}
