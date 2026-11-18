package frc.robot.subsystems.grabber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

/**
 * Constants shared by both the left and right grabber.
 * Side-specific constants (CAN ID, inversion, gear ratio, PID) live in
 * LeftGrabberConstants and RightGrabberConstants.
 */
public class GrabberConstants {

    // all constants are etheir made up or taken form refrence and are subject to change

    // CAN bus both grabber motors are on
    public static final CANBus kCANBus = CANConstants.kRioBus; // TODO: check if these are on the CANivore or the rio bus

    // current limits
    public static final double kSupplyCurrentLimit = 40;
    public static final double kStatorCurrentLimit = 80;

    public static final double kPeakForwardVoltage = 12; //TODO check this.
    public static final double kPeakReverseVoltage = -12;

    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    public static final boolean kUseFOC = true;

    // Applied voltage above this counts as "the motor is running" for the voltageApplied triggers
    public static final double kVoltageAppliedThreshold = 0.1;

    // Preset voltages
    public static final double kIntakeVolts = 6; // TODO: tune this value
    public static final double kOuttakeVolts = -6; // TODO: tune this value
}
