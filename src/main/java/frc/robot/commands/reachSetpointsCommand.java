package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.OutTakeSubsystem;

public class reachSetpointsCommand extends Command {
    private final OutTakeSubsystem shoot;
    private final ArmSubsystem arm;
    private final double[] setpoints;
 // This is the allowed error between the target and actual speeds

    public reachSetpointsCommand(ArmSubsystem arm, OutTakeSubsystem shoot, double[] setpoints) {
        this.shoot = shoot;
        this.setpoints = setpoints;
        this.arm = arm;
        
        addRequirements(arm , shoot);

    }

    @Override
    public void initialize() {
        // Start the outtake motors at the given speed
    }

    @Override
    public void execute() {
        // Execution is handled in initialize, nothing needed here unless you're checking conditions continuously
    }

    @Override
    public boolean isFinished() {
        // return false;
        // Check if both outtake motors are within tolerance of the target speed
        double currentRightSpeed = shoot.getRightMotorVelocity();
        double currentLeftSpeed = shoot.getLeftMotorVelocity();
        boolean shootReady = Math.abs(currentRightSpeed - setpoints[1]) <= Constants.Shooter.tolerance && Math.abs(currentLeftSpeed + setpoints[2]) <= Constants.Shooter.tolerance;
        // boolean armReady = arm.isAtSetpoint(setpoints[3]);
         return shootReady ;//&& armReady;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the outtake motors
        // outTakeSubsystem.stopOutTake()
    }
}
