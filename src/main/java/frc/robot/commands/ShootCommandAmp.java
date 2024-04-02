package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.OutTakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CANdleSystem;

public class ShootCommandAmp extends SequentialCommandGroup {
    public ShootCommandAmp(ArmSubsystem arm, IndexSubsystem index, OutTakeSubsystem shoot, SwerveSubsystem drivebase,
            CANdleSystem led) {
        Pose2d currentPose = drivebase.getPose();
        // double[] setpoints = Constants.getDummyPresets(currentPose.getX(),
        // currentPose.getY());
        addCommands(
                new InstantCommand(() -> {
                    drivebase.limelightForceOnBlink(1);
                }),
                new InstantCommand(() -> {
                    led.setWhite();
                }, led),
                // new InstantCommand(() -> {led.setBlue();}),

                // new WaitUntilCommand(() -> drivebase.hasTarget(1)),
                new ParallelCommandGroup(
                        new ParallelRaceGroup(
                            new OutTakeCommand(shoot, 75, 65, 10),
                            new WaitCommand(1.5)
                        ),
                        new ParallelRaceGroup(
                                new armAbsoluteShoot(drivebase, arm),
                                new WaitUntilCommand(() -> (arm.isAtSetpoint(drivebase, 0.05))),
                                new WaitCommand(2.5)),
                        new ParallelRaceGroup(
                                new WaitUntilCommand(() -> (drivebase.getLimelightX(1) < 0.005)),
                                new WaitCommand(1.5))
                ),

                new InstantCommand(() -> index.runIndex(-1)),

                // drivebase.driveToPose(new Pose2d(currentPose.getTranslation() , new
                // Rotation2d(Math.toRadians(setpoints[0])))),
                // shoot.runOutTakeCommand(setpoints[1], setpoints[2]),
                // arm.setArmPositionCommand(setpoints[1]),
                // new reachSetpointsCommand(arm, shoot ,setpoints)
                new InstantCommand(() -> {
                    led.setGreen();
                }, led),
                new InstantCommand(() -> {
                    drivebase.limelightForceOn(1);
                }),
                new WaitCommand(0.25),
                new InstantCommand(() -> index.stopIndex()),
                new InstantCommand(() -> shoot.stopOutTake()),
                new armAbsoluteControl(arm, Constants.Arm.CANRest),
                new InstantCommand(() -> {
                    led.setPink();
                }, led),

                new InstantCommand(() -> {
                    drivebase.limelightOff(1);
                })

        );

    }
}
