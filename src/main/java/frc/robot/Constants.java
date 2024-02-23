// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(27, 0, 0.05 );

    public static final PIDConstants ANGLE_PID   = new PIDConstants(7, 0, 0);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static final class Shooter{
    public static final double tolerance = 150;
  }
  public static final class IntakeConstants {
  public static final double INTAKE_IN = 1; //Intake Speed
  public static final double INDEX_IN = 0.3; //Index Speed In
  public static final double INDEX_OUT = 1; //Index Speed Out
  public static final double OUTTAKE_OUT = 4500; //OutTake Speed Out
  public static final double OUTTAKE_IN = 5000; //OutTakE Speed In
  public static final double INDEX_DURATION = 1; //Index Duration Time
  public static final double INDEX_OUT_TESTING = -0.3; //INDEX OUT TESTING
  }

  public static final class Arm{
      public static final double Rest = 7; 
      public static final double Intake_Subwoofer = 14; 
      public static final double Wing = 20; 
      public static final double Amp = 30; 
      public static final double Distance = 25;
      
      public static final double kP = 0.035;
      public static final double kI = 0;
      public static final double kD = 0.1;
      public static final double kF = 0;


  }
}
