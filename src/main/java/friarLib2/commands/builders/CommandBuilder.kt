package friarLib2.commands.builders

import edu.wpi.first.wpilibj2.command.*

/**
 * Interface that requires overriding the unary plus operator for
 * edu.wpi.first.wpilibj2.command.Command
 */
interface CommandBuilder {
    operator fun Command.unaryPlus() {}
    operator fun Command.unaryMinus() {}
    fun buildCommand(): Command
}

class SequentialCommandGroupBuilder: SequentialCommandGroup(), CommandBuilder {
    override operator fun Command.unaryPlus() =
        addCommands(this)

    override fun buildCommand(): Command = this
}

class ParallelCommandGroupBuilder: ParallelCommandGroup(), CommandBuilder {
    override operator fun Command.unaryPlus() =
        addCommands(this)

    override fun buildCommand(): Command = this
}

class ParallelRaceGroupBuilder: ParallelRaceGroup(), CommandBuilder {
    override operator fun Command.unaryPlus() =
        addCommands(this)

    override fun buildCommand(): Command = this
}

class ParallelDeadlineGroupBuilder():
    ParallelDeadlineGroup(WaitCommand(0.0)),
    CommandBuilder
{
    override operator fun Command.unaryPlus() =
        addCommands(this)

    /** Use the unary minus operator to add the deadline */
    override operator fun Command.unaryMinus() =
        setDeadline(this)

    override fun buildCommand(): Command = this
}
