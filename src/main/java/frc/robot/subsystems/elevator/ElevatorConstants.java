// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class ElevatorConstants {
    // CAN configs (Kraken X60)
    public static final int kMotorCANId = 31; // TODO: set real CAN ID
    public static final CANBus kCANBus = CANConstants.kRioBus;

    // Voltage and current limits
    public static final double kMaxVoltage = 12;
    public static final double kSupplyCurrentLimit = 40;
    public static final double kStatorCurrentLimit = 80;

    public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive; // TODO: confirm positive = up
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    public static final boolean kUseFOC = true;

    // Mechanism
    public static final double kGearRatio = 5; // 5:1 gearbox, rotor rotations per output shaft rotation
    public static final double kSprocketPitchDiameterMeters = 0.0447; // TODO: measure (this is a 22T #25 sprocket)
    public static final double kRiggingRatio = 1; // Single stage: carriage moves 1:1 with the chain/belt
    /** Carriage travel in meters per output shaft rotation. Used as the TalonFXUnitConverter conversion factor. */
    public static final double kMetersPerRotation = Math.PI * kSprocketPitchDiameterMeters * kRiggingRatio;

    // Closed-loop (MotionMagicVoltage, so output units are volts and SI units are meters)
    public static final double kP = 0; // TODO: tune this value
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0; // TODO: tune this value
    public static final double kG = 0; // TODO: tune this value
    public static final double kV = 0; // TODO: tune this value
    public static final double kA = 0;

    // Motion Magic profile
    public static final double kCruiseVelocityMetersPerSec = 1.0; // TODO: tune this value
    public static final double kMaxAccelMetersPerSec2 = 2.0; // TODO: tune this value
    public static final double kMaxJerkMetersPerSec3 = 20.0; // TODO: tune this value

    // Limits
    public static final double kMinHeightMeters = 0;
    public static final double kMaxHeightMeters = 1.0; // TODO: measure max travel
    public static final double kHeightToleranceMeters = 0.02;

    public static final double kMaxOperatorControlVolts = 3;

    /** Preset heights for the elevator */
    public static enum ElevatorPositions {
        STOW(0), // TODO: set real heights
        LOW(0.3),
        MID(0.6),
        HIGH(0.9);

        public final double heightMeters;

        private ElevatorPositions(double heightMeters) {
            this.heightMeters = heightMeters;
        }
    }
}
