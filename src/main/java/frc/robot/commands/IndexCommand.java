package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;

public class IndexCommand extends Command {
    private final IndexSubsystem IndexCommand;
    private final double speed;
    private final double duration;
    private long startTime;

    public IndexCommand(IndexSubsystem index, double speed, double duration) {
        this.IndexCommand = index;
        this.speed = speed;
        this.duration = duration;
        addRequirements(IndexCommand);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        IndexCommand.runIndex(speed);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) >= (duration * 1000);
    }

    @Override
    public void end(boolean interrupted) {
        IndexCommand.stopIndex();
    }
}
