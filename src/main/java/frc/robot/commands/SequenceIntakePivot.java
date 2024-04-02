package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CANdleSystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SequenceIntakePivot extends SequentialCommandGroup {
    public SequenceIntakePivot(IntakeSubsystem inTakeSubsystem, IndexSubsystem indexSubsystem, double indexSpeed, double indexDuration, CANdleSystem led) {
        addCommands(
            new IntakeIndexCommand(inTakeSubsystem, indexSubsystem, indexDuration, indexSpeed , led),
            new WaitUntilCommand(() -> indexSubsystem.getLimit()),
            new IndexCommand(indexSubsystem, indexSpeed, Constants.IntakeConstants.INDEX_DURATION),
            new InstantCommand(() -> inTakeSubsystem.stopIntake()), 
            new InstantCommand(() -> indexSubsystem.stopIndex()),  
            //new WaitCommand(1),
            new InstantCommand(() -> {led.BaseLEDLights();})

        );
    }
}
