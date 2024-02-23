package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OutTakeSubsystem;

public class OutTakeCommand extends Command {
    private final OutTakeSubsystem outTakeSubsystem;
    private final double speed;
    private final double speed1;

    private final double tolerance; // This is the allowed error between the target and actual speeds

    public OutTakeCommand(OutTakeSubsystem intake, double speed1,double speed, double tolerance) {
        this.outTakeSubsystem = intake;
        this.speed = speed;
        this.speed1 = speed1;

        this.tolerance = tolerance;
        addRequirements(outTakeSubsystem);
    }

    @Override
    public void initialize() {
        // Start the outtake motors at the given speed
        outTakeSubsystem.runOutTake(speed1, speed);
    }

    @Override
    public void execute() {
        // Execution is handled in initialize, nothing needed here unless you're checking conditions continuously
    }

    @Override
    public boolean isFinished() {
        // return false;
        // Check if both outtake motors are within tolerance of the target speed
        double currentRightSpeed = outTakeSubsystem.getRightMotorVelocity();
        double currentLeftSpeed = outTakeSubsystem.getLeftMotorVelocity();
        return Math.abs(currentRightSpeed - speed1) <= tolerance && Math.abs(currentLeftSpeed + speed) <= tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the outtake motors
        // outTakeSubsystem.stopOutTake()
    }
}
