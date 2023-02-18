package frc.robot.commands.turntable;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Turntable;
import frc.robot.subsystems.TurntableSubsystem;

public class SpinTurnTable extends CommandBase {
    private TurntableSubsystem turntable;

    public SpinTurnTable(TurntableSubsystem turntable) {
        this.turntable = turntable;

        addRequirements(turntable);
    }

    @Override
    public void initialize() {
        turntable.startTurntable();
    }

    @Override
    public void end(boolean interrupted) {
        turntable.stopTurntable();
    }
}
