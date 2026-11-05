// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.SystemConstants.RobotMode;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drum.DrumIODisabled;
import frc.robot.subsystems.drum.DrumIOKraken;
import frc.robot.subsystems.drum.DrumSubsystem;
import frc.robot.subsystems.elevator.ElevatorIODisabled;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberIODisabled;
import frc.robot.subsystems.grabber.GrabberIOKraken;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.indexer.IndexerIODisabled;
import frc.robot.subsystems.indexer.IndexerIOKraken;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeIODisabled;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeSubsystem;
import lib.hardware.hid.SamuraiXboxController;

import com.ctre.phoenix6.CANBus.CANBusStatus;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final GrabberSubsystem m_grabber;
    private final ElevatorSubsystem m_elevator;
    private final IntakeSubsystem m_intake;
    private final IndexerSubsystem m_indexer;
    private final DrumSubsystem m_drum;

    private final Elastic m_elastic;

    // Controllers
    private final SamuraiXboxController m_driverController =
        new SamuraiXboxController(OIConstants.kDriverControllerPort);
    private final SamuraiXboxController m_operatorController = 
        new SamuraiXboxController(OIConstants.kOperatorControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        if (SystemConstants.kMode == RobotMode.REAL || RobotBase.isReal()) {
            // The robot is real! Use the real IO layers
            m_grabber = new GrabberSubsystem(new GrabberIOKraken());
            m_elevator = new ElevatorSubsystem(new ElevatorIOKraken());
            m_intake = new IntakeSubsystem(new IntakeIOKraken());
            m_indexer = new IndexerSubsystem(new IndexerIOKraken());
            m_drum = new DrumSubsystem(new DrumIOKraken());
        } else {
            // The robot is not real. Use the disabled IO layers
            m_grabber = new GrabberSubsystem(new GrabberIODisabled());
            m_elevator = new ElevatorSubsystem(new ElevatorIODisabled());
            m_intake = new IntakeSubsystem(new IntakeIODisabled());
            m_indexer = new IndexerSubsystem(new IndexerIODisabled());
            m_drum = new DrumSubsystem(new DrumIODisabled());
        }

        m_elastic = new Elastic();

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
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(m_exampleSubsystem::exampleCondition)
                .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        m_operatorController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

        // Update alliance from Elastic
        new Trigger(() -> m_elastic.getSelectedTeamColor() == Alliance.Blue)
            .onTrue(Commands.runOnce(() -> setAlliance(Alliance.Blue)).ignoringDisable(true));
        new Trigger(() -> m_elastic.getSelectedTeamColor() == Alliance.Red)
            .onTrue(Commands.runOnce(() -> setAlliance(Alliance.Red)).ignoringDisable(true));
    }

    /** Bind triggers on driver controller to commands */
    private void configureDriverBindings() {

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
