// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static double[] getDummyPresets(double x, double y) { // less dummy know
    
    double deltaY = y-5.5; 
    double heading = Units.radiansToDegrees(Math.atan( deltaY / x)) ;
    double armAngle = Units.radiansToDegrees(Math.atan(1.39/ (Math.sqrt( Math.pow(x , 2) - Math.pow(deltaY , 2)))) );
    double conv = 0.00523;
    double[] a = { heading, armAngle, 3 };
    return a;
  }

  public static final class AprilTag {
    public final static AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public enum ID {
      kBlueSourceRight(1),
      kBlueSourceLeft(2),
      kRedSpeakerRight(3),
      kRedSpeakerCenter(4),
      kRedAmp(5),
      kBlueAmp(6),
      kBlueSpeakerCenter(7),
      kBlueSpeakerLeft(8),
      kRedSourceRight(9),
      kRedSourceLeft(10),
      kRedStageLeft(11),
      kRedStageRight(12),
      kRedStageCenter(13),
      kBlueStageCenter(14),
      kBlueStageLeft(15),
      kBlueStageRight(16);

      private final int id;

      private ID(int id) {
        this.id = id;
      }

      public int getID() {
        return id;
      }
    }
  }

  public static final class Vision {
    public static final String VISION_CAMERA = "VISION_CAMERA";

    public static final String DASHBOARD_PREFIX = "vision/";

    public static final Transform3d CAMERA_POSITION = new Transform3d(
        Units.inchesToMeters(13.5),
        0,
        Units.inchesToMeters(20),
        new Rotation3d(0, Units.degreesToRadians(-20), 0));
  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.3, 0.01, 0);

    public static final PIDConstants ANGLE_PID = new PIDConstants(0.51, 0, 0.001);
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.2;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class Shooter {
    public static final double tolerance = 10;
  }

  public static final class IntakeConstants {
    public static final double INTAKE_IN = 1; // Intake Speed
    public static final double INDEX_IN = 0.3; // Index Speed In
    public static final double INDEX_OUT = 1; // Index Speed Out
    public static final double OUTTAKE_OUT = 1000;//4500; // OutTake Speed Out
    public static final double OUTTAKE_IN = 1000; //5000; // OutTakE Speed In
    public static final double INDEX_DURATION = 1; // Index Duration Time
    public static final double INDEX_OUT_TESTING = -0.3; // INDEX OUT TESTING
  }

  public static final class Arm {
    public static final class CAN {

        public static final double kP = 60;
        public static final double kI = 3;

        public static final double kD = 0;
    }
    public static final int CANcoder = 5;
    public static final double CANconv = 0.005422;
    public static double conv = 2/3;
    public static final double CAN90 = -0.09203125;//-0.052724609375;// -0.018798828125;//0.051025390625; //0.07275390625; // 0.1953125; //2.15;//-0.0452931640625;//-0.028076171875; 
    

    public static final double CANRest = CAN90 - 0.03; // 0.67626953125;
        public static final double CANWing2 = CAN90 - 0.1221484375; //0.517333984375;

    public static final double CANWing = CAN90 - 0.1381484375; //0.517333984375;
    public static final double CANAmp = CAN90 - 0.52865234375; //0.32568359375; 


    public static final double Rest = 5;
    public static final double Intake_Subwoofer = 14;
    public static final double Wing = 20;
    public static final double Amp = 5 + (90* 2/3); 
    public static final double Distance = 25;

    public static final double kP = 0.15;//0.08;
    public static final double kI = 0;
    public static final double kD = 0.5;
    public static final double kF = 0;

  }

  public static final class NotePID {
    public static final double kP = .012;
    public static final double kI = 0.0;
    public static double kD = 0;

  }
  public static final class RotatePID{
    public static final double kP = .0075;
    public static final double kI= 0.0;
    public static double kD = 0;

  }
  public static final class Driver {
    public static final int leftXAxis = 0;
    public static final int leftYAxis = 1;
    public static final int rightXAxis = 2;
    public static final int rightYAxis = 5;

    public static final int shootButton = 5;
    public static final int intakeButton = 6;
    public static final int autoAlignButton = 6;
    public static final int robotFieldButton = 7;

  }

    public static final class LED {

    public static final int LEDCount = 68;
    public static final int leftYAxis = 1;


  }
}
