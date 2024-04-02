package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

public class armAbsoluteControlEND extends Command{
 // Boolean driveMode;
    ArmSubsystem arm;
    double setpoint;
    double startTime;

public armAbsoluteControlEND( ArmSubsystem arm, double setpoint)
{

this.arm = arm;
this.setpoint = setpoint;
// Use addRequirements() here to declare subsystem dependencies.
addRequirements(arm);
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(arm.getAbsolutePID(setpoint));
    arm.runPercent(arm.getAbsolutePID(setpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //  return false;
    return arm.getAbsoluteAtSetpoint();
//    return (System.currentTimeMillis() - startTime) >= (2 * 1000);
  }

}

