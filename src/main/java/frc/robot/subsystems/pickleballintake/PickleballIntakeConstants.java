package frc.robot.subsystems.pickleballintake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class PickleballIntakeConstants {
    public static final int kCANID = 34; //TODO correct CAN stuff
    public static final CANBus kCanBus = CANConstants.kCANivoreBus;

    /** Velocity gains in SI units (volts per radian per second) */
    public static class Control {
        public static final double kP = 0.0;//TODO correct PID controler values
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static final double kPeakForwardVoltage = 12;//TODO check these values.
    public static final double kPeakReverseVoltage = -12;
    public static final double kSupplyCurrentLimit = 40;
    public static final double kStatorCurrentLimit = 70;
    public static final double kGearRatio = 1;
    public static final InvertedValue kInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    public static final boolean kUseFOC = true;

    // Applied voltage above this counts as "the intake is running"
    public static final double kVoltageAppliedThreshold = 0.1;

    // Preset voltages
    public static final double kIntakeVolts = 6; // TODO: tune this value
    public static final double kReverseVolts = -4; // TODO: tune this value
}
