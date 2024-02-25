// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.alignToAprilTag;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.HandleNoteSequence;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeIndexCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexSubsystem;
// import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OutTakeSubsystem;
// import frc.robot.subsystems.OutTakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.limelight;
import swervelib.SwerveDrive;

import java.io.File;

import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
 {
     private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final IndexSubsystem indexSubsystem = new IndexSubsystem();
    private final OutTakeSubsystem outTakeSubsystem = new OutTakeSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();

    private final limelight limelight = new limelight();


  //     private final OutTakeSubsystem outakeSubsystem = new OutTakeSubsystem();


  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));


  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);
  Joystick driverXbox = new Joystick(0);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    //                                                                () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
    //                                                                                             OperatorConstants.LEFT_Y_DEADBAND),
    //                                                                () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
    //                                                                                             OperatorConstants.LEFT_X_DEADBAND),
    //                                                                () -> MathUtil.applyDeadband(driverXbox.getRightX(),
    //                                                                                             OperatorConstants.RIGHT_X_DEADBAND),
    //                                                                driverXbox::getYButtonPressed,
    //                                                                driverXbox::getAButtonPressed,
    //                                                                driverXbox::getXButtonPressed,
    //                                                                driverXbox::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(4),
        () -> driverXbox.getRawAxis(5));


        // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        //   () -> MathUtil.applyDeadband(driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND), // Left Stick Y-Axis
        //   () -> MathUtil.applyDeadband(driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND), // Left Stick X-Axis
        //   () -> MathUtil.applyDeadband(driverXbox.getRawAxis(4), OperatorConstants.RIGHT_X_DEADBAND) // Right Stick X-Axis
        //   );
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(4),
        () -> !(driverXbox.getRawAxis(2) > 0.5)); 

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
          () -> MathUtil.applyDeadband(driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND), // Left Stick Y-Axis
          () -> MathUtil.applyDeadband(driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(4));

  drivebase.setDefaultCommand(
      !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
    // armSubsystem.setDefaultCommand(armSubsystem.armCommand(() -> driverXbox.getRawAxis(3) , () -> driverXbox.getRawAxis(2)));
    // SwerveDrive.setHeadingCorrection(true);
    }
    
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(driverXbox, 10).onTrue((new InstantCommand(drivebase::zeroGyro)));
    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox, 5).onTrue(new InstantCommand(() -> intakeSubsystem.runIntake(IntakeConstants.Intake_Speed)));
    // new JoystickButton(driverXbox, 5).onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));
   new JoystickButton(driverXbox, 2).onTrue(new InstantCommand(() -> armSubsystem.setArmPosition(Constants.Arm.Rest)));
      
   new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(() -> armSubsystem.setArmPosition(Constants.Arm.Wing)));
       
   new JoystickButton(driverXbox, 7).onTrue(new InstantCommand(() -> armSubsystem.setEncoder(Constants.Arm.Rest)));
  // new JoystickButton(driverXbox, 9).whileTrue((new alignToAprilTag()));
   new JoystickButton(driverXbox, 9).whileTrue((new alignToAprilTag(drivebase, 1)));

  
   // new JoystickButton(driverXbox, 6).toggleOnTrue(new IntakeIndexCommand(intakeSubsystem, indexSubsystem, 1, 0.5));
   new JoystickButton(driverXbox, 1).toggleOnTrue(new HandleNoteSequence(outTakeSubsystem, indexSubsystem, Constants.IntakeConstants.OUTTAKE_IN, Constants.IntakeConstants.OUTTAKE_OUT, Constants.IntakeConstants.INDEX_OUT,Constants.IntakeConstants.INDEX_DURATION ));
 //  new JoystickButton(driverXbox, 6).toggleOnTrue(new HandleNoteSequence(intakeSubsystem, indexSubsystem, 1, 0.5));
   new JoystickButton(driverXbox, 6).toggleOnTrue(new IntakeIndexCommand(intakeSubsystem, indexSubsystem, Constants.IntakeConstants.INTAKE_IN, Constants.IntakeConstants.INDEX_IN));
      
   new JoystickButton(driverXbox, 4).onTrue(new InstantCommand(() -> outTakeSubsystem.runOutTake(Constants.IntakeConstants.OUTTAKE_IN , Constants.IntakeConstants.OUTTAKE_IN)));
   new JoystickButton(driverXbox, 4).onFalse(new InstantCommand(() -> outTakeSubsystem.stopOutTake()));

   new JoystickButton(driverXbox, 5).onTrue(new InstantCommand(() -> indexSubsystem.runIndex(-0.5)));
    new JoystickButton(driverXbox, 5).onFalse(new InstantCommand(() -> indexSubsystem.stopIndex()));
    
    // new JoystickButton(driverXbox, 1).onTrue(new InstantCommand(() -> outTakeSubsystem.runOutTake(4500, 4500))); //nspeed 4500 + P 5000
    // new JoystickButton(driverXbox, 1).onFalse(new InstantCommand(() -> outakeSubsystem.stopIntake()));



//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
      return new PathPlannerAuto("Example Autoo");

  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
