package friarLib2.commands.builders

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup

class ConditionalCommandBuilder(
    var condition: () -> Boolean = { println("No condition added to conditional command"); true }
) : CommandBuilder {

    private val onTrue = ParallelCommandGroup()
    private val onFalse = ParallelCommandGroup()

    /**
     * Add a command to run when the condition is true
     */
    override fun Command.unaryPlus() {
        onTrue.addCommands(this)
    }

    /**
     * Add a command to run when the condition is false
     */
    override fun Command.unaryMinus() {
        onFalse.addCommands(this)
    }

    /**
     * Set the condition for the command
     */
    operator fun (() -> Boolean).unaryPlus() {
        condition = this
    }

    override fun buildCommand(): Command =
        ConditionalCommand(onTrue, onFalse, condition)
}