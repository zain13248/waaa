package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class output extends Command {
    private final SwerveSubsystem drive;
    // private final double speed;
    private final double duration;
    private long startTime;

    public output( SwerveSubsystem drive , double duration) {
        this.duration = duration;
        this.drive = drive;
        // addRequirements(IndexCommand);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();
        double[] setpoints = Constants.getDummyPresets(currentPose.getX(), currentPose.getY());
    
        System.out.println("Set" + setpoints[0]);
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) >= (duration * 1000);
    }

    @Override
    public void end(boolean interrupted) {
        // IndexCommand.stopIndex();
    }
}
