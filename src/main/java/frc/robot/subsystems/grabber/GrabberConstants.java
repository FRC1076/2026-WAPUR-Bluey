// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.grabber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class GrabberConstants {
    // CAN configs
    public static final int kLeadMotorCANId = 21; // TODO: set real CAN ID
    public static final int kFollowMotorCANId = 22; // TODO: set real CAN ID
    public static final CANBus kCANBus = CANConstants.kRioBus;

    // Voltage and current limits
    public static final double kMaxVoltage = 12;
    public static final double kSupplyCurrentLimit = 40;
    public static final double kStatorCurrentLimit = 60;

    public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive; // TODO: confirm direction
    public static final MotorAlignmentValue kFollowerAlignment = MotorAlignmentValue.Opposed; // Motors are mirrored
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    // Gear ratio
    public static final double kGearRatio = 1; // TODO: confirm gear ratio

    // Closed-loop (VelocityTorqueCurrentFOC, so output units are amps)
    public static final double kP = 0; // TODO: tune this value
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0; // TODO: tune this value
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double kVelocityFilterTimeConstant = 0.01;

    // Preset voltages
    public static final double kIntakeVolts = 6; // TODO: tune this value
    public static final double kHoldVolts = 1; // TODO: tune this value
    public static final double kEjectVolts = -6; // TODO: tune this value

    // FOC
    public static final boolean kUseFOC = true;
}
