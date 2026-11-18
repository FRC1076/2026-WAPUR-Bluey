// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.SystemConstants.RobotMode;
import frc.robot.commands.PickleballCommands;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.drum.DrumIODisabled;
import frc.robot.subsystems.drum.DrumIOKraken;
import frc.robot.subsystems.drum.DrumSubsystem;
import frc.robot.subsystems.elevator.ElevatorIODisabled;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.left.LeftGrabberIODisabled;
import frc.robot.subsystems.grabber.left.LeftGrabberIOKraken;
import frc.robot.subsystems.grabber.left.LeftGrabberSubsystem;
import frc.robot.subsystems.grabber.right.RightGrabberIODisabled;
import frc.robot.subsystems.grabber.right.RightGrabberIOKraken;
import frc.robot.subsystems.grabber.right.RightGrabberSubsystem;
import frc.robot.subsystems.indexer.IndexerIODisabled;
import frc.robot.subsystems.indexer.IndexerIOKraken;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.pickleballintake.PickleballIntakeIODisabled;
import frc.robot.subsystems.pickleballintake.PickleballIntakeIOKraken;
import frc.robot.subsystems.pickleballintake.PickleballIntakeSubsystem;
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
    private final LeftGrabberSubsystem m_leftGrabber;
    private final RightGrabberSubsystem m_rightGrabber;
    private final ElevatorSubsystem m_elevator;
    private final PickleballIntakeSubsystem m_pickleballIntake;
    private final IndexerSubsystem m_indexer;
    private final DrumSubsystem m_drum;

    private final VisionLocalizationSystem m_vision;

    private final Elastic m_elastic;

    // Command for the driver
    private final TeleopDriveCommand teleopDriveCommand;

    // Controllers
    private final SamuraiPS5Controller m_driverController =
        new SamuraiPS5Controller(OIConstants.kDriverControllerPort);
    private final SamuraiXboxController m_operatorController =
        new SamuraiXboxController(OIConstants.kOperatorControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_vision = new VisionLocalizationSystem();
        m_elastic = new Elastic();

        if (SystemConstants.kMode == RobotMode.REAL && RobotBase.isReal()) {
            // The robot is real! Use the real IO layers
            m_drive = new DriveSubsystem(
                new DriveIOHardware(TunerConstants.createDrivetrain()),
                m_vision,
                m_elastic
            );
            m_leftGrabber = new LeftGrabberSubsystem(new LeftGrabberIOKraken());
            m_rightGrabber = new RightGrabberSubsystem(new RightGrabberIOKraken());
            m_elevator = new ElevatorSubsystem(new ElevatorIOKraken());
            m_pickleballIntake = new PickleballIntakeSubsystem(new PickleballIntakeIOKraken());
            m_indexer = new IndexerSubsystem(new IndexerIOKraken());
            m_drum = new DrumSubsystem(new DrumIOKraken());
        } else {
            // The robot is not real. Use the disabled or simulation IO layers
            m_drive = new DriveSubsystem(
                new DriveIOSim(TunerConstants.createDrivetrain()),
                m_vision,
                m_elastic
            );
            m_leftGrabber = new LeftGrabberSubsystem(new LeftGrabberIODisabled());
            m_rightGrabber = new RightGrabberSubsystem(new RightGrabberIODisabled());
            m_elevator = new ElevatorSubsystem(new ElevatorIODisabled());
            m_pickleballIntake = new PickleballIntakeSubsystem(new PickleballIntakeIODisabled());
            m_indexer = new IndexerSubsystem(new IndexerIODisabled());
            m_drum = new DrumSubsystem(new DrumIODisabled());
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

    /** Use this method to define bindings that are not tied to a specific controller */
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

        // How to pick the binding type:
        //   whileTrue -> runs while the button is held, stops when released (grabbers, pickleball, brake)
        //   onTrue    -> runs once when the button is pressed (elevator levels, reset gyro)

        // Brake: L3 or R3                       -> teleopDriveCommand.applySwerveBrake()   (whileTrue)
        // Reset gyro: Options                    -> Commands.runOnce(() -> m_drive.resetHeading())   (onTrue)
        // Grabber intake/outtake left: L2 / L1   -> m_leftGrabber.intake() / m_leftGrabber.outtake()   (whileTrue)
        // Grabber intake/outtake right: R2 / R1  -> m_rightGrabber.intake() / m_rightGrabber.outtake()   (whileTrue)
        // Pickleball shooter: D-pad up           -> PickleballCommands.shoot(m_drum, m_indexer)   (whileTrue)
        // Pickleball intake: D-pad down          -> m_pickleballIntake.intake()   (whileTrue)
        // Elevator levels 1-4: Cross / Circle / Square / Triangle -> m_elevator.goToLevel(1) ... goToLevel(4)   (onTrue)
    }

    /** Bind triggers on operator controller to commands */
    private void configureOperatorBindings() {
        // Pickleball shooter backwards: D-pad up  -> PickleballCommands.shootBackwards(m_drum, m_indexer)   (whileTrue)
        // Pickleball intake backwards: D-pad down -> m_pickleballIntake.reverse()   (whileTrue)
        // Elevator increment up: right stick up   -> m_elevator.incrementUp()   (onTrue)
        //     Stick up is negative Y, so the trigger is: new Trigger(() -> m_operatorController.getRightY() < -0.5)
        // Elevator increment down: right stick click -> m_elevator.incrementDown()   (onTrue)
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
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