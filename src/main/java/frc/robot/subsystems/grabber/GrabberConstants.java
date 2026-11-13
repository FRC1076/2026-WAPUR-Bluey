package frc.robot.subsystems.grabber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class GrabberConstants {

    // all constants are etheir made up or taken form refrence and are subject to change
    
    // CAN ID
    public static final int kRightMotorCANID = 0; // TODO: set this to the correct CAN ID
    public static final int kLeftMotorCANID = 0;
    public static final CANBus kCANBus = CANConstants.kCANivoreBus;

    public class RightControl {
        //PID constants for right motor
        public static final double kP = 0.0; // TODO: tune these values
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        public static final double kVelocityTimeFilterSecs = 0.0;
    } 

    public class LeftControl {
        //PID constants for left motor 
        public static final double kP = 0.0; // TODO: tune these values
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        public static final double kVelocityTimeFilterSecs = 0.0;
    }

    //voltage compensation limits and current limits 
    public static final double kVoltageCompensation = 12;
    public static final double kSupplyCurrentLimit = 40;
    public static final double kStatorCurrentLimit = 70;

    public static final double kPeakForwardVoltage = 12; //TODO check this.
    public static final double kPeakReverseVoltage = -12;

    public static final InvertedValue kRightInverted = InvertedValue.CounterClockwise_Positive; //TODO check this.
    public static final InvertedValue kLeftInverted = InvertedValue.Clockwise_Positive;

    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
    
    public static final int kLeftGearRatio = 1;
    public static final int kRightGearRatio = 1;

    public static final boolean kUseFOC = true;





}
