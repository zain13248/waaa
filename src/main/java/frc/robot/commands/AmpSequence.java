package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.OutTakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CANdleSystem;


public class AmpSequence extends SequentialCommandGroup {
    public AmpSequence (IndexSubsystem index , OutTakeSubsystem shoot, ArmSubsystem arm, CANdleSystem led, SwerveSubsystem drivebase ){
    // double[] setpoints = Constants.getDummyPresets(currentPose.getX(), currentPose.getY());
    addCommands(
         new InstantCommand(() -> {drivebase.limelightForceOnBlink(0);}) ,
         new InstantCommand(() -> {led.setWhite();} , led),
         new InstantCommand(() -> index.runIndex(0.1)),
        new WaitCommand(0.05),
         new InstantCommand(() -> index.stopIndex()),

       //  new InstantCommand(() -> {Constants.Vision.VISION_CAMERA();}),
         new ParallelCommandGroup(
            new ParallelRaceGroup(
            new OutTakeCommand(shoot , 40, 40, 15),
            new WaitCommand(1)
            ),
            new armAbsoluteControl(arm, Constants.Arm.CANAmp)),
            // new InstantCommand( () -> arm.setArmPosition(Constants.Arm.CANAmp) ) ),

         new InstantCommand(() -> index.runIndex(-1) ),
        // new InstantCommand(() -> {led.setGreen();}),

            // new rotateToTarget(drivebase, setpoints[0]),
            //  drivebase.driveToPose(new Pose2d(currentPose.getTranslation() , new Rotation2d(Math.toRadians(setpoints[0])))),
            //  shoot.runOutTakeCommand(setpoints[1], setpoints[2]),
            //  arm.setArmPositionCommand(setpoints[1]),
            // new reachSetpointsCommand(arm, shoot ,setpoints)
        new InstantCommand(() -> {led.setGreen();} , led),

        new WaitCommand(.3),
        new InstantCommand( () -> index.stopIndex()),
        new InstantCommand( () -> shoot.stopOutTake()),
        new armAbsoluteControl(arm, Constants.Arm.CANRest),
        new WaitCommand(.5),
         new InstantCommand(() -> {led.setPink();} ,led)

        // new RunCommand(led::BaseLEDLights , led)

        // new InstantCommand( () -> arm.setArmPosition(Constants.Arm.CANRest) )
     
        // new armAbsoluteControlEND (arm, Constants.Arm.CANRest)


    );

    }
}
