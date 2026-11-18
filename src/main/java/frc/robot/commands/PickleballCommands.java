// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drum.DrumConstants;
import frc.robot.subsystems.drum.DrumSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;

/**
 * The "pickleball shooter" is the drum and the indexer working together.
 * All of these commands run while held and stop everything when released (use whileTrue).
 */
public final class PickleballCommands {
    private PickleballCommands() {}

    /**
     * Spins the drum up, then feeds pickleballs into it with the indexer once the drum is
     * up to speed (or after {@link DrumConstants#kMaxSpinUpSeconds}, whichever comes first).
     */
    public static Command shoot(DrumSubsystem drum, IndexerSubsystem indexer) {
        return drum.spinUp().alongWith(
            Commands.waitUntil(drum::atSetpoint)
                .withTimeout(DrumConstants.kMaxSpinUpSeconds)
                .andThen(indexer.index())
        );
    }

    /** Runs the drum and indexer backwards to push pickleballs back out */
    public static Command shootBackwards(DrumSubsystem drum, IndexerSubsystem indexer) {
        return drum.reverse().alongWith(indexer.reverse());
    }
}
