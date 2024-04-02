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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommand;
//import frc.robot.commands.SequencePivotShooter;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeIndexCommand;
//import frc.robot.commands.ShootCommandSpeaker;
import frc.robot.commands.ShootCommandAmp;
import frc.robot.commands.ShootCommandAuto;

//import frc.robot.commands.ShootCommandSpeaker;
import frc.robot.commands.OutTakeCommand;
import frc.robot.commands.IntakeIndexCommandNoStop;
import frc.robot.commands.AmpSequence;

import frc.robot.commands.alignToAprilTag;
import frc.robot.commands.alignToNote;
import frc.robot.commands.allignToAmp;
import frc.robot.commands.armAbsoluteControl;
import frc.robot.commands.armAbsoluteControlEND;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.IndexSubsystem;
// import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OutTakeSubsystem;
// import frc.robot.subsystems.ClimberSubsystem;

// import frc.robot.subsystems.OutTakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.limelight;
import swervelib.SwerveDrive;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

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
  //  private final ClimberSubsystem climbsubsystem = new ClimberSubsystem();
    // private final limelight limelight = new limelight();


  //     private final OutTakeSubsystem outakeSubsystem = new OutTakeSubsystem();


  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo") );


  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  XboxController opJoystick = new XboxController(1);
  XboxController driverXbox = new XboxController(0);
      private final CANdleSystem m_candleSubsystem = new CANdleSystem(driverXbox , indexSubsystem);
        private String m_autoSelected;
      private final SendableChooser<Command> m_chooser = new SendableChooser<>();
      private final Command FourNoteAuto = new PathPlannerAuto("4 Note Auto Tested");


       


  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    getAutonomousCommand();
    configureBindings();

      


      m_chooser.setDefaultOption("Four Note Auto", FourNoteAuto);

    SmartDashboard.putData("Auto Mode", m_chooser);
    NamedCommands.registerCommand("intakeRun", new IntakeIndexCommand(intakeSubsystem, indexSubsystem, -1, -0.22 , m_candleSubsystem
    ));

   // PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("Path1Test");

    NamedCommands.registerCommand("shoot", new ShootCommandAuto( indexSubsystem , outTakeSubsystem));

    NamedCommands.registerCommand("intakeRunNoStop", new IntakeIndexCommandNoStop(intakeSubsystem, indexSubsystem, -1, -1));
    NamedCommands.registerCommand("setIntake", new InstantCommand(()->intakeSubsystem.runIntake(-1)));
    
    NamedCommands.registerCommand("spool", new InstantCommand(()->outTakeSubsystem.runOutTake(60,50)));

    NamedCommands.registerCommand("arm_mid", new armAbsoluteControl( armSubsystem , Constants.Arm.CANWing));
    NamedCommands.registerCommand("arm_rest", new armAbsoluteControl( armSubsystem , Constants.Arm.CANRest , 0.05));

    NamedCommands.registerCommand("stop", new ParallelCommandGroup(new InstantCommand(()->outTakeSubsystem.stopOutTake()) , new InstantCommand( ()-> intakeSubsystem.stopIntake())));

    NamedCommands.registerCommand("NoteDetection", new alignToNote(drivebase , indexSubsystem));


        // NamedCommands.registerCommand("ShootLeft", new ShootCommandAuto(drivebase, indexSubsystem, outTakeSubsystem, armSubsystem, 45));
        // NamedCommands.registerCommand("ShootMiddle", new ShootCommandAuto(drivebase, indexSubsystem, outTakeSubsystem, armSubsystem, 0));
        // NamedCommands.registerCommand("ShootRight", new ShootCommandAuto(drivebase, indexSubsystem, outTakeSubsystem, armSubsystem, -45));


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
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> driverXbox.getRawAxis(4),
    //     () -> driverXbox.getRawAxis(5));


    //     Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //       () -> MathUtil.applyDeadband(driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND), // Left Stick Y-Axis
    //       () -> MathUtil.applyDeadband(driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND), // Left Stick X-Axis
    //       () -> MathUtil.applyDeadband(driverXbox.getRawAxis(4), OperatorConstants.RIGHT_X_DEADBAND) // Right Stick X-Axis
    //       );
    // // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(4),OperatorConstants.RIGHT_X_DEADBAND) ,
        () -> driverXbox.getRawAxis(2) , 
        () -> driverXbox.getRawButton(6) , 
        () -> (driverXbox.getRawAxis(3) >0.1),
        () -> driverXbox.getRawButton(4)
        );

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
          () -> MathUtil.applyDeadband(driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND), // Left Stick Y-Axis
          () -> MathUtil.applyDeadband(driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(4));
  //climbsubsystem.setDefaultCommand( climbsubsystem.climbCommand(() -> MathUtil.applyDeadband(opJoystick.getRawAxis(1) , 0.1)));
  drivebase.setDefaultCommand( driveFieldOrientedAnglularVelocity);
     armSubsystem.setDefaultCommand(armSubsystem.armCommand(() ->MathUtil.applyDeadband( opJoystick.getRawAxis(5) , 0.1)  ,
       () -> opJoystick.getRawButton(1 ) , () -> opJoystick.getRawButton(2 ),   () -> opJoystick.getRawButton(3 ) ));
    // m_candleSubsystem.setDefaultCommand(new RunCommand(m_candleSubsystem::setPink , m_candleSubsystem));
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
    new POVButton(driverXbox, 0).onTrue(new ParallelCommandGroup(
        new InstantCommand(()->outTakeSubsystem.stopOutTake(), outTakeSubsystem), 
        new InstantCommand(()->intakeSubsystem.stopIntake() , intakeSubsystem),
                new InstantCommand(()->indexSubsystem.stopIndex() , indexSubsystem)

         ));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger( () -> (driverXbox.getRawAxis(3) > 0.5)).onTrue( new ShootCommandAmp(armSubsystem,indexSubsystem , outTakeSubsystem , drivebase , m_candleSubsystem) );
    //new POVButton(driverXbox, 270).onTrue(new allignToAmp(drivebase));

    new JoystickButton(driverXbox, 7).onTrue((new InstantCommand(drivebase::zeroGyro)));
        // new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::getPose1)));
    // new JoystickButton(driverXbox, 1).onTrue((new alignToAprilTag(drivebase ,  1)));

    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox, 5).onTrue(new InstantCommand(() -> intakeSubsystem.runIntake(IntakeConstants.Intake_Speed)));
    // new JoystickButton(driverXbox, 5).onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));
 //  new JoystickButton(driverXbox, 2).onTrue(new InstantCommand(() -> armSubsystem.setArmPosition(Constants.Arm.Rest)));
    //  new JoystickButton(driverXbox, 4).whileTrue(new InstantCommand(() -> intakeSubsystem.runIntake(-1)));
     //    new JoystickButton(driverXbox, 4).onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));

 //  new JoystickButton(driverXbox, 3).toggleOnTrue(new InstantCommand(() -> drivebase.resetOdometry(new Pose2d(new Translation2d( 0,0) , new Rotation2d(0)))));
       
  //  new JoystickButton(opJoystick, 1).whileTrue(new InstantCommand( () -> armSubsystem.setArmPosition(Constants.Arm.CANRest) )); 
  //     new JoystickButton(opJoystick, 2).whileTrue(new InstantCommand( () -> armSubsystem.setArmPosition(Constants.Arm.CANWing) )); 
  //  new JoystickButton(opJoystick, 3).whileTrue(new InstantCommand( () -> armSubsystem.setArmPosition(Constants.Arm.CANAmp) )); 

   
  //  new JoystickButton(opJoystick, 2).whileTrue(new armAbsoluteControl(armSubsystem, Constants.Arm.CANWing)); 
  //  new JoystickButton(opJoystick, 3).whileTrue(new armAbsoluteControl(armSubsystem, Constants.Arm.CANAmp)); 
  //  new JoystickButton(opJoystick, 4).whileTrue(new InstantCommand(() -> armSubsystem.setEncoder(0))); 

  //  new JoystickButton(dri                                           verXbox , 1).toggleOnTrue(new ShootCommand(drivebase, indexSubsystem, outTakeSubsystem, armSubsystem));
  // new JoystickButton(driverXbox, 9).whileTrue((new alignToAprilTag()));
//   new JoystickButton(driverXbox, 9).whileTrue((new alignToAprilTag(drivebase, 1)));




      // new JoystickButton(driverXbox, 1).onTrue(new InstantCommand(() ->intakeSubsystem.runIntake(-1)));
      //       new JoystickButton(driverXbox, 1).onFalse(new InstantCommand(() ->intakeSubsystem.stopIntake()));

      //     new JoystickButton(driverXbox, 1).onTrue(new InstantCommand(() ->indexSubsystem.runIndex(-0.5)));
      //           new JoystickButton(driverXbox, 1).onFalse(new InstantCommand(() ->indexSubsystem.stopIndex()));


                
      // new JoystickButton(driverXbox, 2).onTrue(new InstantCommand(() ->intakeSubsystem.runIntake(-1)));
      //       new JoystickButton(driverXbox, 2).onFalse(new InstantCommand(() ->intakeSubsystem.stopIntake()));

      //     new JoystickButton(driverXbox, 2).onTrue(new InstantCommand(() ->indexSubsystem.runIndex(0.5)));
      //           new JoystickButton(driverXbox, 2).onFalse(new InstantCommand(() ->indexSubsystem.stopIndex()));


    // new JoystickButton(driverXbox, 6).onTrue(new IntakeIndexCommand(intakeSubsystem, indexSubsystem, 1, -0.5));
    // new JoystickButton(driverXbox, 6).onFalse(new IntakeIndexCommand(intakeSubsystem, indexSubsystem, 1, 0.5));

    // new JoystickButton(driverXbox, 2).onTrue(new IntakeIndexCommand(intakeSubsystem, indexSubsystem, -1, -0.5));
    // new JoystickButton(driverXbox, 2).onFalse(new IntakeIndexCommand(intakeSubsystem, indexSubsystem, -1, -0.5));

 //  new JoystickButton(driverXbox, 1).toggleOnTrue(new HandleNoteSequence(outTakeSubsystem, indexSubsystem, Constants.IntakeConstants.OUTTAKE_IN, Constants.IntakeConstants.OUTTAKE_OUT, Constants.IntakeConstants.INDEX_OUT,Constants.IntakeConstants.INDEX_DURATION ));
 //  new JoystickButton(driverXbox, 6).toggleOnTrue(new HandleNoteSequence(intakeSubsystem, indexSubsystem, 1, 0.5));
            
  new JoystickButton(driverXbox, 6).onTrue(new ParallelCommandGroup(  new armAbsoluteControl(armSubsystem, Constants.Arm.CANRest), new IntakeIndexCommand(intakeSubsystem, indexSubsystem, -1, -0.1 , m_candleSubsystem) ));
  // new JoystickButton(driverXbox, 6).onTrue(new InstantCommand(() -> m_candleSubsystem.setOrange()));
// 
  // new JoystickButton(driverXbox, 4).whileTrue(new IntakeIndexCommand(intakeSubsystem, indexSubsystem, 1, 0.15 , m_candleSubsystem));

  // new JoystickButton(driverXbox, 4).whileTrue(new alignToAprilTag(drivebase, 8));

  //  new JoystickButton(driverXbox, 4).onFalse(new InstantCommand(() -> outTakeSubsystem.stopOutTake()));
  // new JoystickButton(driverXbox, 1).onTrue(new ShootCommandSpeaker(drivebase, indexSubsystem, outTakeSubsystem, armSubsystem, m_candleSubsystem));

//new JoystickButton(driverXbox, 4).onTrue(new InstantCommand(() -> outTakeSubsystem.runOutTake(4000, 4500) ));
// new JoystickButton(driverXbox, 4).onTrue(new allignToAmp(drivebase));
new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(() -> indexSubsystem.runIndex(-1)));

new JoystickButton(driverXbox, 2).onTrue(new ParallelCommandGroup(new InstantCommand(() -> indexSubsystem.runIndex(0.2)) , new InstantCommand( () -> intakeSubsystem.runIntake(1))));
// new JoystickButton(driverXbox, 2).onTrue(new InstantCommand(() -> intakeSubsystem.runIntake(1)));


// new JoystickButton(driverXbox, 2).onTrue(new InstantCommand(() -> outTakeSubsystem.runOutTakePercent(1)));

//new JoystickButton(driverXbox, 7).onTrue(new InstantCommand(() -> climbsubsystem.runClimber(0.2)));
new JoystickButton(driverXbox, 8).onTrue(new OutTakeCommand(outTakeSubsystem, 75, 65, 10));
//new JoystickButton(driverXbox, 7).onFalse(new InstantCommand(() -> outTakeSubsystem.stopOutTake()));


new JoystickButton(driverXbox, 5).onTrue(new  AmpSequence ( indexSubsystem ,  outTakeSubsystem,  armSubsystem,  m_candleSubsystem,  drivebase ));
// new JoystickButton(driverXbox, 5).onTrue(new InstantCommand(() -> m_candleSubsystem.setGreen()));



    //  new JoystickButton(opJoystick, 1).toggleOnTrue(new RunCommand(m_candleSubsystem::BaseLEDLights, m_candleSubsystem));
    // new JoystickButton(opJoystick, 2).toggleOnTrue(new RunCommand(m_candleSubsystem::incrementAnimation, m_candleSubsystem));
    // new JoystickButton(opJoystick, 3).toggleOnTrue(new RunCommand(m_candleSubsystem::decrementAnimation, m_candleSubsystem));

    // new JoystickButton(driverXbox, 10).whileTrue( drivebase.driveToPose(new Pose2d(
      // drivebase.getPose().getTranslation().getX()+1 , drivebase.getPose().getTranslation().getY()+1  , drivebase.getPose().getRotation())
      // ));


    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(() -> outTakeSubsystem.runOutTake(4000, 6800))); //nspeed 4000 + P 6800
    // new JoystickButton(driverXbox, 3).onFalse(new InstantCommand(() -> outTakeSubsystem.stopOutTake()));





    // Operator Joystick Commands




  //    new JoystickButton(opJoystick, 1).onTrue(new InstantCommand(() -> indexSubsystem.runIndex(1))); //OUT SPEED ONLY
  //    new JoystickButton(opJoystick, 1).onFalse(new InstantCommand(() -> indexSubsystem.stopIndex()));

  //    //new JoystickButton(opJoystick, 1).onFalse(new InstantCommand(() -> indexSubsystem.stopIndex()));

  // //  new JoystickButton(opJoystick, 2).onTrue(new InstantCommand(() -> intakeSubsystem.runIntake(1)));
  //     new JoystickButton(opJoystick, 3).onTrue(new InstantCommand(() -> intakeSubsystem.runIntake(1))); //OUT SPEED ONLY
  //   new JoystickButton(opJoystick, 3).onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));

  //   new JoystickButton(driverXbox, 4).onTrue(new InstantCommand(() -> outTakeSubsystem.stopOutTake()));  // STOP OUTAKE
  //   new JoystickButton(driverXbox, 4).onFalse(new InstantCommand(() -> outTakeSubsystem.stopOutTake()));



// Operator Joystick Commands





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
    //  return new Command() {};
    return new PathPlannerAuto("Side Note Bottom");
    //return new SequentialCommandGroup(new InstantCommand(drivebase::zeroGyro) , m_chooser.getSelected());
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();0
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  public void setAmpSetpoint(){
    drivebase.setAmpSetpoint();
  }

}
