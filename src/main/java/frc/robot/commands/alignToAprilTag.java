package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

public class alignToAprilTag extends Command {
  /** Creates a new aprilTagSwerve. */
  Double tx,ty,ta;
 // Boolean driveMode;
  SwerveSubsystem swerve;
  SwerveController controller;
  double kP,kI,kD;
  PIDController thetaController;
  int targetTag;

public alignToAprilTag(SwerveSubsystem swerve, int targetTag)
{

this.swerve = swerve;
this.controller = swerve.getSwerveController();
// Use addRequirements() here to declare subsystem dependencies.
addRequirements(swerve);
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {



    thetaController = new PIDController(Constants.NotePID.kP, Constants.NotePID.kI, Constants.NotePID.kD);
   // swerve.limelightForceOn(1);
    getLimelightValues();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getLimelightValues();
    printLimelightVal(); // -0.1
    swerve.drive(new Translation2d(-0.5, 0),
    (thetaController.calculate(swerve.getLimelightX(0),0) * controller.config.maxAngularVelocity), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.limelightOff(0);
        swerve.drive(new Translation2d(
                 0,
                 0),
                 0,
                 false);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if(Math.abs(tx) > 1 || Math.abs(ta) < 4 ){

      return false;
    }else return true;

   // return (System.currentTimeMillis() - startTime) >= (2 * 1000);
  }



  public void getLimelightValues()
  {
    tx= swerve.getLimelightX(0);
    ty = swerve.getLimelightY(0);
    ta = swerve.getLimelightA(0);
  }

  public void printLimelightVal()
  {
    getLimelightValues();
    SmartDashboard.putNumber("Limelight tx", tx);
    SmartDashboard.putNumber("Limelight ty", ty);
    SmartDashboard.putNumber("Limelight ta", ta);
  }
}