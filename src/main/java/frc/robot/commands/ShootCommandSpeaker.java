// package frc.robot.commands;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.IndexSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.OutTakeSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.CANdleSystem;

// public class ShootCommandSpeaker extends SequentialCommandGroup {
//     public ShootCommandSpeaker (SwerveSubsystem drivebase , IndexSubsystem index , OutTakeSubsystem shoot , ArmSubsystem arm, CANdleSystem led){
//     Pose2d currentPose = drivebase.getPose();
//     double[] setpoints = Constants.getDummyPresets(currentPose.getX(), currentPose.getY());
//     addCommands(
//         new InstantCommand(() -> {drivebase.limelightForceOnBlink(1);}) ,

//         new ParallelRaceGroup(
//             new WaitCommand(5),
//             new rotateToTarget(drivebase, setpoints[0]),
//             //  drivebase.driveToPose(new Pose2d(currentPose.getTranslation() , new Rotation2d(Math.toRadians(setpoints[0])))),
//               shoot.runOutTakeCommand(setpoints[1], setpoints[2]),
//              // arm.setArmPositionCommand(setpoints[2]),
//             new reachSetpointsCommand(arm, shoot, setpoints)
//         ),

//         new IndexCommand(index, Constants.IntakeConstants.INDEX_OUT, Constants.IntakeConstants.INDEX_DURATION),
//         new InstantCommand(() -> {drivebase.limelightForceOn(1);}) ,
//         new WaitCommand(1), 
//         new InstantCommand(() -> {drivebase.limelightOff(1);}),
//         new InstantCommand(() -> {led.BaseLEDLights();})

//     );

//     }

//     public ShootCommandSpeaker(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem, int i, double d) {
//         //TODO Auto-generated constructor stub
//     }
// }
