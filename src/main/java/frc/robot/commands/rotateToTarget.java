package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class rotateToTarget extends Command {
    SwerveSubsystem swerve;
    double setpoint;

    public rotateToTarget(SwerveSubsystem swerve)
{
    this.swerve = swerve;
    addRequirements(swerve);
}
public void initialize() {
    
   Pose2d currentPose = swerve.getPose();
   double y = currentPose.getY();
   double x = currentPose.getX();
   double deltaY = y-5.5; 
    this.setpoint = 45;// Units.radiansToDegrees(Math.atan( deltaY / x)) ;


}
public void execute() {
    System.out.println("Heading " + setpoint);
    swerve.rotateToTarget(setpoint);
}
public boolean isFinished(){
    if(Math.abs(setpoint - swerve.getHeading().getDegrees() ) > 0.5
     ){ // || Math.abs(ta) < 4
        return false;
      }else return true;
}
}
