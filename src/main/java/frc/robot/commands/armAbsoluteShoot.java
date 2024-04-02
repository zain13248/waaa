package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

public class armAbsoluteShoot extends Command{
 // Boolean driveMode;
    ArmSubsystem arm;
    SwerveSubsystem swerve;


public armAbsoluteShoot( SwerveSubsystem  swerve, ArmSubsystem arm)
{
this.swerve = swerve;
this.arm = arm;

// Use addRequirements() here to declare subsystem dependencies.
addRequirements(arm);
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        Pose2d currentPose = swerve.getPose();

   double y = currentPose.getY();
   double x = currentPose.getX();
   double deltaY = y-5.5; 
   double setpoint = arm.getSetpoint(swerve);
    // -0.1969 +(0.0927*Math.log(swerve.getLimelightA(1))) ;
    if(! swerve.hasTarget(1)){
      setpoint = Constants.Arm.CANRest;
    }
  //  double setpoint = Constants.Arm.CAN90 - ((55-Units.radiansToDegrees(Math.atan(0.5/swerve.getLimelightA(1) )) )  * Constants.Arm.CANconv ) ;
    // System.out.println("hiii" + setpoint);
        SmartDashboard.putNumber("ARM Setpoint", setpoint);

    SmartDashboard.putBoolean("ARM Aiming", true);

   arm.setArmPosition(setpoint);//Constants.Arm.CANRest);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      SmartDashboard.putBoolean("ARM Aiming", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return !swerve.hasTarget(1);
//    return (System.currentTimeMillis() - startTime) >= (2 * 1000);
  }

}

