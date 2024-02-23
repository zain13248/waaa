
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToSetpointCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final double setpoint;

    public MoveArmToSetpointCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        this.setpoint = 50; 

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setArmPosition(setpoint);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtSetpoint(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Stop the arm movement if the command is interrupted
            armSubsystem.stopArm();
        }
        // You could also optionally hold the position here if desired
    }
}
