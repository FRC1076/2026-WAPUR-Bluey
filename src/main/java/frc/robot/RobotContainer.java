// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.SystemConstants.RobotMode;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.TunerConstants;
import lib.hardware.hid.SamuraiPS5Controller;
import lib.hardware.hid.SamuraiXboxController;
import lib.vision.VisionLocalizationSystem;

import com.ctre.phoenix6.CANBus.CANBusStatus;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_drive;

    private final VisionLocalizationSystem m_vision;

    private final Elastic m_elastic;

    // Command for the driver
    TeleopDriveCommand teleopDriveCommand;

    // Controllers
    private final SamuraiPS5Controller m_driverController =
        new SamuraiPS5Controller(OIConstants.kDriverControllerPort);
    private final SamuraiXboxController m_operatorController = 
        new SamuraiXboxController(OIConstants.kOperatorControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_vision = new VisionLocalizationSystem();
        m_elastic = new Elastic();

        if (SystemConstants.kMode == RobotMode.REAL || RobotBase.isReal()) {
            // The robot is real! Use the real IO layers
            m_drive = new DriveSubsystem(
                new DriveIOHardware(TunerConstants.createDrivetrain()),
                m_vision,
                m_elastic
            );
        } else {
            // The robot is not real. Use the disabled or simulation IO layers
            m_drive = new DriveSubsystem(
                new DriveIOSim(TunerConstants.createDrivetrain()),
                m_vision,
                m_elastic
            );
        }

        teleopDriveCommand = new TeleopDriveCommand(
            m_drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()
        );

        // Set the alliance color
        setAlliance(m_elastic.getSelectedTeamColor());

        // Configure the trigger bindings
        configureBindings();

        configureDriverBindings();

        configureOperatorBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Update alliance from Elastic
        new Trigger(() -> m_elastic.getSelectedTeamColor() == Alliance.Blue)
            .onTrue(Commands.runOnce(() -> setAlliance(Alliance.Blue)).ignoringDisable(true));
        new Trigger(() -> m_elastic.getSelectedTeamColor() == Alliance.Red)
            .onTrue(Commands.runOnce(() -> setAlliance(Alliance.Red)).ignoringDisable(true));
    }

    /** Bind triggers on driver controller to commands */
    private void configureDriverBindings() {
        m_drive.setDefaultCommand(teleopDriveCommand);

        // Single clutch
        m_driverController.R1().and(m_driverController.R2().negate())
            .whileTrue(teleopDriveCommand.applySingleClutch());

        m_driverController.R2()
            .whileTrue(teleopDriveCommand.applyDoubleClutch());

        // Brake
        m_driverController.L3().or(m_driverController.R3())
            .whileTrue(teleopDriveCommand.applySwerveBrake());

        // Reset the heading of the gyro
        m_driverController.create()
            .onTrue(Commands.runOnce(() ->
                m_drive.resetHeading()
            ));
    }

    /** Bind triggers on operator controller to commands */
    private void configureOperatorBindings() {

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return m_elastic.getSelectedAutonomousCommand();
    }

    public void registerNamedCommands() {

    }

    public CANBusStatus getCANivoreBusStatus() {
        return CANConstants.kCANivoreBus.getStatus();
    }

    /** Command to raise thread priority */
    public static Command threadCommand() {
        return Commands.sequence(
            Commands.waitSeconds(25),
            Commands.runOnce(() -> Threads.setCurrentThreadPriority(true, 1)),
            Commands.print("Main Thread Priority raised to RT1 at " + Timer.getFPGATimestamp())
        ).ignoringDisable(true);
    }

    /** Command to change which alliance we are on */
    public void setAlliance(Alliance newAlliance) {
        // m_drive.setAllianceRotation(newAlliance);
        // m_superstructure.setTeamColor(newAlliance);

        // TODO: uncomment when these exist
    }
}
