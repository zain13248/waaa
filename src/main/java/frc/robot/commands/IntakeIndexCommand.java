package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;


public class IntakeIndexCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
        private final IndexSubsystem indexSubsystem;
        private final CANdleSystem LED;
    

    private final double intakeSpeed;
    private final double indexSpeed; 
        public IntakeIndexCommand(IntakeSubsystem intake, IndexSubsystem index ,  double intakeSpeed, double indexSpeed , CANdleSystem LED) {
        this.intakeSubsystem = intake;
        this.intakeSpeed = intakeSpeed;
        this.indexSpeed = indexSpeed;
        this.indexSubsystem = index;
        this.LED = LED;
        addRequirements(intakeSubsystem , indexSubsystem );
    
    }
    @Override
    public void initialize() {
        // armSubsystem.setArmPosition(Constants.Arm.Res);
    }

    @Override
    public void execute() {
        // if(indexSubsystem.indexCurrent()){
        //     LED.setOrange();
        // }
        // else{
        //     LED.setPink();
        // }
        intakeSubsystem.runIntake(intakeSpeed);
        indexSubsystem.runIndex(indexSpeed);

    }

    @Override
    public void end(boolean interrupted) {
                //     LED.setOrange();

        intakeSubsystem.stopIntake();
        indexSubsystem.stopIndex();
        LED.setPink();
    }

    @Override
    public boolean isFinished() {
        return indexSubsystem.getLimit(); 
    }    
    
}
