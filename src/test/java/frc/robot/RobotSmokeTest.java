package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Smoke test: starts the real Robot class in simulation ONCE, then runs it for five seconds in
 * teleop followed by five seconds in autonomous. Any exception thrown during robot construction or
 * from the periodic loops fails the test.
 *
 * <p>Robot is only constructed once per JVM because AdvantageKit's Logger, the CommandScheduler,
 * NetworkTables and similar singletons cannot be safely re-created between tests. Both modes are
 * therefore covered in a single test by switching the simulated Driver Station mode mid-run.
 *
 * <p>Place this file at src/test/java/frc/robot/RobotSmokeTest.java.
 */
class RobotSmokeTest {
    private static final double SECONDS_PER_MODE = 5.0;

    private volatile Robot robot;
    private Thread robotThread;
    private final AtomicReference<Throwable> robotError = new AtomicReference<>();

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
        DriverStationSim.resetData();
    }

    @AfterEach
    void tearDown() throws InterruptedException {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        if (robot != null) {
            robot.endCompetition();
        }
        if (robotThread != null) {
            robotThread.join(2000);
        }
        if (robot != null) {
            robot.close();
        }
        DriverStationSim.resetData();
        HAL.shutdown();
    }

    /** Puts the simulated DS into the requested mode and enables the robot. */
    private static void setMode(boolean autonomous) {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.setAutonomous(autonomous);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    /** Lets the robot run for the given time, failing fast if it crashes or its loop exits. */
    private void runFor(String modeName, double seconds) throws InterruptedException {
        long end = System.nanoTime() + (long) (seconds * 1e9);
        while (System.nanoTime() < end) {
        if (robotError.get() != null || !robotThread.isAlive()) {
            break;
        }
        Thread.sleep(50);
        }

        Throwable error = robotError.get();
        if (error != null) {
        throw new AssertionError("Robot threw an exception during " + modeName, error);
        }
        assertTrue(
            robotThread.isAlive(),
            "Robot loop exited before the " + seconds + "s of " + modeName + " were up");
    }

    @Test
    @Timeout(60)
    void teleopThenAutonomousRunWithoutErrors() throws InterruptedException {
        setMode(false); // start in teleop

        // Run the robot's main loop on its own thread, the same way RobotBase.startRobot would.
        robotThread =
            new Thread(
                () -> {
                try {
                    robot = new Robot();
                    robot.startCompetition();
                } catch (Throwable t) {
                    robotError.set(t);
                }
                },
                "robot-main");
        robotThread.setUncaughtExceptionHandler((th, t) -> robotError.set(t));
        robotThread.start();

        runFor("teleop", SECONDS_PER_MODE);

        // Switch to autonomous on the same running robot instance.
        setMode(true);
        runFor("autonomous", SECONDS_PER_MODE);
    }
}