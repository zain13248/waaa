// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.SwerveSubsystem;

// public class driveToTarget extends Command {
//     SwerveSubsystem swerve;
//     double x;
//     double y;

//     public driveToTarget(SwerveSubsystem swerve, double x, double y)
// {
//     this.y = y;
//     this.x = x;
//     this.swerve = swerve;
//     addRequirements(swerve);
// }
// public void initialize() {
// double[] presets = Constants.getDummyPresets(x, y);
// double heading = presets[0];
// double armAngle = presets[1];
// }

// public void execute() {

//    // swerve.rotateToTarget(setpoint);
// }
// public boolean isFinished(){
//     if(Math.abs(setpoint - swerve.getHeading().getDegrees() ) > 0.5
//      ){ // || Math.abs(ta) < 4
//         return false;
//       }else return true;
// }
// }
