package friarLib2.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand

/**
 * A command that starts after a certain amount of time and ends after
 * some amount of time.
 *
 * <p>If duration is zero, then don't add a timeout for the timed
 * command, letting it run until it finishes by itself.
 */
class TimedCommand(
    startTime: Double,
    duration: Double,
    command: Command
): SequentialCommandGroup() {
    init {
        if (startTime != 0.0) {
            addCommands(WaitCommand(startTime))
        }

        if (duration != 0.0) {
            addCommands(command.withTimeout(duration))
        } else {
            addCommands(command)
        }
    }
}
