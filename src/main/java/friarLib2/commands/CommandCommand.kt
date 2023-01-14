package friarLib2.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase

open class CommandCommand(val command: Command): CommandBase() {
    init {
        for (requirement in command.requirements) {
            addRequirements(requirement)
        }
    }

    override fun initialize() =
            command.initialize()

    override fun execute() =
            command.execute()

    override fun end(interrupted: Boolean) =
            command.end(interrupted)

    override fun isFinished(): Boolean =
            command.isFinished()
}