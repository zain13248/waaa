package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.OutTakeSubsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class HandleNoteSequence extends SequentialCommandGroup {
    public HandleNoteSequence(OutTakeSubsystem outtakeSubsystem, IndexSubsystem indexSubsystem, double outtakeSpeed1, double outtakeSpeed, double indexSpeed, double indexDuration) {
        addCommands(
            new WaitUntilCommand(() -> indexSubsystem.getLimit()),
            new OutTakeCommand(outtakeSubsystem, outtakeSpeed1, outtakeSpeed1, Constants.Shooter.tolerance),
            new IndexCommand(indexSubsystem, indexSpeed, Constants.IntakeConstants.INDEX_DURATION),
            new InstantCommand(() -> outtakeSubsystem.stopOutTake()), 
            new InstantCommand(() -> indexSubsystem.stopIndex())

        );
    }
}
