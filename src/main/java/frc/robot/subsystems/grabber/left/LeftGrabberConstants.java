package frc.robot.subsystems.grabber.left;

import com.ctre.phoenix6.signals.InvertedValue;

public class LeftGrabberConstants {

    // all constants are etheir made up or taken form refrence and are subject to change

    // CAN ID
    public static final int kMotorCANID = 31; // TODO: set this to the correct CAN ID (must be different from the right grabber's)

    public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive; //TODO check this.

    public static final double kGearRatio = 1; // TODO: set this to the real gear ratio

    public static final double kVelocityFilterTimeConstant = 0.0;

    /** Gains for velocity control (TalonFX slot 0), in SI units (volts per radian) */
    public static class VelocityControl {
        public static final double kP = 0.0; // TODO: tune these values
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    /** Gains for position control (TalonFX slot 1), in SI units (volts per radian) */
    public static class PositionControl {
        public static final double kP = 0.0; // TODO: tune these values
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
    }
}
