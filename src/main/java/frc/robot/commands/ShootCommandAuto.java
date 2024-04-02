package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.OutTakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CANdleSystem;


public class ShootCommandAuto extends SequentialCommandGroup {
    public ShootCommandAuto (IndexSubsystem index , OutTakeSubsystem shoot){
    // double[] setpoints = Constants.getDummyPresets(currentPose.getX(), currentPose.getY());
    addCommands(
       //  new InstantCommand(() -> {drivebase.limelightForceOnBlink(1);}) ,
        //  new armCommand(() -> {arm.(1);}) ,
        // new InstantCommand(() -> {led.setBlue();}),
        new ParallelRaceGroup(
            new OutTakeCommand(shoot , 70, 60, 10),
            new WaitCommand(0.75)),

        new InstantCommand(() -> index.runIndex(-1) ),
            // new rotateToTarget(drivebase, setpoints[0]),
            //  drivebase.driveToPose(new Pose2d(currentPose.getTranslation() , new Rotation2d(Math.toRadians(setpoints[0])))),
            //  shoot.runOutTakeCommand(setpoints[1], setpoints[2]),
            //  arm.setArmPositionCommand(setpoints[1]),
            // new reachSetpointsCommand(arm, shoot ,setpoints)

        new WaitCommand(0.1),
        new InstantCommand( () -> index.stopIndex()) 


    );

    }
}
