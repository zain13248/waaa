// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.IndexSubsystem;

// public class OuTakeIndexCommand extends Command {
//     private final IndexSubsystem indexSubsystem;
//     private final double speed;
//     private final double duration;
//     private long startTime;

//     public OuTakeIndexCommand(IndexSubsystem index, double speed, double duration) {
//         this.indexSubsystem = index;
//         this.speed = speed;
//         this.duration = duration;
//         addRequirements(indexSubsystem);
//     }

//     @Override
//     public void initialize() {
//         startTime = System.currentTimeMillis();
//         indexSubsystem.runIndex(speed);
//     }

//     @Override
//     public void execute() {
//     }

//     @Override
//     public boolean isFinished() {
//         return (System.currentTimeMillis() - startTime) >= (duration * 1000);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         indexSubsystem.stopIndex();
//     }
// }
