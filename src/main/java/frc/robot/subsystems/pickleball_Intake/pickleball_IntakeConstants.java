package frc.robot.subsystems.pickleball_Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.CANConstants;

public class pickleball_IntakeConstants{
    public static final int kCANID = 0; //TODO correct CAN stuff
    public static final CANBus kCanBus = CANConstants.kCANivoreBus;

    public class Control {
        public static final double kP = 0.0;//TODO correct PID controler values 
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static final double kPeakForwardVoltage = 12;//TODO check these values.
    public static final double kPeakReverseVoltage = -12;
    public static final double kVoltageCompensation = 12;
    public static final double kSupplyCurrentLimit = 40;
    public static final double kStatorCurrentLimit = 70;
    public static final double kGearRatio = 1;
    public static final InvertedValue kInvertedValue = InvertedValue.CounterClockwise_Positive
    
    public static final boolean kUseFOC = true;

 
}