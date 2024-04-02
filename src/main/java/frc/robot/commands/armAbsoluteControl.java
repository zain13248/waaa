package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

public class armAbsoluteControl extends Command{
 // Boolean driveMode;
    ArmSubsystem arm;
    double setpoint;
    double startTime;
    double tolerance = 0.01;

public armAbsoluteControl( ArmSubsystem arm, double setpoint)
{

this.arm = arm;
this.setpoint = setpoint;
// Use addRequirements() here to declare subsystem dependencies.
addRequirements(arm);
}

public armAbsoluteControl( ArmSubsystem arm, double setpoint, double tolerance)
{

this.arm = arm;
this.setpoint = setpoint;
this.tolerance = tolerance;
// Use addRequirements() here to declare subsystem dependencies.
addRequirements(arm);
}
// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    arm.setArmPosition(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//    return false;
    // double tolerance = 0.01;
    if(setpoint == Constants.Arm.CANAmp){
        tolerance = 0.075;
    }
    return arm.isAtSetpoint(setpoint , tolerance);
//    return (System.currentTimeMillis() - startTime) >= (2 * 1000);
  }

}

