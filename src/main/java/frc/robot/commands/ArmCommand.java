package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
    private final ArmSubsystem ArmSubsystem;
    private final double position;


    public ArmCommand(ArmSubsystem intake,double position) {
        this.ArmSubsystem = intake;
        this.position = position;

        addRequirements(ArmSubsystem);
    }

    @Override
    public void initialize() {
        // ArmSubsystem.setArmPosition(position);
        // Start the outtake motors at the given speed
        // ArmSubsystem.runArm(speed);
    }

    @Override
    public void execute() {
        // Execution is handled in initialize, nothing needed here unless you're checking conditions continuously
    }

    @Override
    public boolean isFinished() {
        return false;
        // return ArmSubsystem.isAtSetpoint(position);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the outtake motors
    }
}
