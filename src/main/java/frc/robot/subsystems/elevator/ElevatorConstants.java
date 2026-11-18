package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANConstants;

public class ElevatorConstants {
    public static final int kLeaderID = 33; // TODO: set the real CAN ID
    public static final CANBus kCANBus = CANConstants.kCANivoreBus; // TODO: check if this is on the CANivore or the rio bus

    // voltage
    public static final double kPeakForwardVoltage = 12;
    public static final double kPeakReverseVoltage = -12;

    // values
    public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive; // depends on orientation
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake; // Stop the elevator from falling upon disable

    // current limits
    public static final double kSupplyCurrentLimit = 40; // TODO: check these
    public static final double kStatorCurrentLimit = 80;

    // magic motion, in meters of carriage travel
    public static final double kCruiseVelocityMetersPerSec = 1000.0; // TODO: tune (fast placeholder so motion magic can move freely)
    public static final double kAccelerationMetersPerSec2 = 1000.0; // TODO: tune (high placeholder so motion magic can move freely)
    public static final double kJerkMetersPerSec3 = 0; // target jerk (0 = no jerk limit)

    // gains in SI units (volts per meter), TODO: tune these values
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // cascade elevator calculations
    public static final double kSensorToMechanismRatio = 5; // TODO: check this value
    public static final double kDrumPitchDiameter = Units.inchesToMeters(1.757); // TODO: check this value
    public static final double kStages = 1; // one stage elevator
    public static final double kMetersPerDrumRotation = kDrumPitchDiameter * Math.PI * kStages;

    // soft limits, in meters
    public static final double kSlackMargin  = 0.1; // leeway in position limits for safety
    public static final double kForwardPositionLimit = 4.5 - kSlackMargin; // TODO: check max height
    public static final double kBackwardPositionLimit = kSlackMargin;

    // level heights, in meters
    public static final double kLevel1HeightMeters = 0.2; // TODO: measure real level 1 height
    public static final double kLevel2HeightMeters = 1.0; // TODO: measure real level 2 height
    public static final double kLevel3HeightMeters = 2.0; // TODO: measure real level 3 height
    public static final double kLevel4HeightMeters = 3.0; // TODO: measure real level 4 height

    // how far one increment up/down moves the elevator, in meters
    public static final double kIncrementMeters = 0.05; // TODO: tune
}
