package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CANdleSystem;

public class IntakeIndexCommandNoStop extends Command {
    private final IntakeSubsystem intakeSubsystem;
        private final IndexSubsystem indexSubsystem;
    private final double intakeSpeed;
    private final double indexSpeed; 
        public IntakeIndexCommandNoStop(IntakeSubsystem intake, IndexSubsystem index , double intakeSpeed, double indexSpeed) {
        this.intakeSubsystem = intake;
        this.intakeSpeed = intakeSpeed;
        this.indexSpeed = indexSpeed;
        this.indexSubsystem = index;
        addRequirements(intakeSubsystem , indexSubsystem);
    
    }
    @Override
    public void initialize() {
        // Nothing to initialize
    }

    @Override
    public void execute() {
        intakeSubsystem.runIntake(intakeSpeed);
        indexSubsystem.runIndex(indexSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        indexSubsystem.stopIndex();
    }

    @Override
    public boolean isFinished() {
        return false; //indexSubsystem.getLimit(); 
    }    
    
}
