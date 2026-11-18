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
import frc.robot.subsystems.drum.DrumIODisabled;
import frc.robot.subsystems.drum.DrumIOKraken;
import frc.robot.subsystems.drum.DrumSubsystem;
import frc.robot.subsystems.elevator.ElevatorIODisabled;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.indexer.IndexerIODisabled;
import frc.robot.subsystems.indexer.IndexerIOKraken;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.grabber.left.LeftGrabberIODisabled;
import frc.robot.subsystems.grabber.left.LeftGrabberIOKraken;
import frc.robot.subsystems.grabber.left.LeftGrabberSubsystem;
import frc.robot.subsystems.grabber.right.RightGrabberIODisabled;
import frc.robot.subsystems.grabber.right.RightGrabberIOKraken;
import frc.robot.subsystems.grabber.right.RightGrabberSubsystem;
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
    private final LeftGrabberSubsystem m_leftGrabber;
    private final RightGrabberSubsystem m_rightGrabber;
    private final ElevatorSubsystem m_elevator;
    private final PickleballIntakeSubsystem m_pickleballIntake;
    private final IndexerSubsystem m_indexer;
    private final DrumSubsystem m_drum;

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

        // Brake
        m_driverController.L3().or(m_driverController.R3())
            .whileTrue(teleopDriveCommand.applySwerveBrake());

        // Pickeball Controls
            m_driverController.povUp()
            .onTrue(Commands.none()) //pickeball shooter on
            .onFalse(Commands.none()); //pickeball shooter off
            
        m_driverController.povDown()
            .onTrue(Commands.none()) //pickleball intake on
            .onFalse(Commands.none()); //pickeball intake off

        // Elevator Controls
        m_driverController.cross()
            .onTrue(Commands.none()); //elevator level 1
        
        m_driverController.circle()
            .onTrue(Commands.none()); //elevator level 2 

        m_driverController.square()
            .onTrue(Commands.none()); //elevator level 3

        m_driverController.triangle()
            .onTrue(Commands.none()); //elevator level 4

        //Grabber Controls
        m_driverController.L1()
            .onTrue(Commands.none()); // grabber outtake left 

        m_driverController.L2()
            .onTrue(Commands.none()); // grabber intake left 

        m_driverController.R1()
            .onTrue(Commands.none()); //grabber outtake right 

        m_driverController.R2()
            .onTrue(Commands.none()); //grabber intake right
        
        

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
