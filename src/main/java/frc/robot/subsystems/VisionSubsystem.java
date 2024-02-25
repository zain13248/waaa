package frc.robot.subsystems;











//THIS IS NOT USED --> LIMELIGHT.java & SwerveSubsystem scroll all the way down 










import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private NetworkTable mainTable;
  public int mode;

  public VisionSubsystem() { // initializes device
    mainTable =
        NetworkTableInstance.getDefault().getTable("limelight"); // gets the network table with key
    // "limelight"
    mode = 0;

    // m_XP = ShuffleboardInfo.getInsatnce().
  }

  public void setLight(boolean on) { // toggles lights (true for on, false for off)
    mainTable.getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  public void setMode(int selection) {
    mode = selection;
    mainTable.getEntry("pipeline").setNumber(selection);
  }

  public int getTagID() { // returns id of apriltag or -1 if no tag is detected.
    double tid = mainTable.getEntry("tid").getDouble(-1);
    SmartDashboard.putNumber("AprilTag ID", tid);
    return (int) tid;
  }

  public boolean tagDetected() { // returns true if tag is detected
    double ttarget = mainTable.getEntry("tv").getDouble(0);
    boolean tdetected = ttarget == 0 ? false : true;
    SmartDashboard.putBoolean("AprilTag Detected", tdetected);
    return tdetected;
  }

  public double getTagArea() { // return tag area
    double ta = mainTable.getEntry("ta").getDouble(0);
    SmartDashboard.putNumber("Tag Area", ta);
    return ta;
  }

  public double getTagX() { // return tag x value (horizontal across camera screen)
    double tx = mainTable.getEntry("tx").getDouble(0);
    SmartDashboard.putNumber("Tag X", tx);
    return tx;
  }

  public double getTagY() { // return tag y value (vertical across camera screen)
    double ty = mainTable.getEntry("ty").getDouble(0);
    SmartDashboard.putNumber("Tag Y", ty);
    return ty;
  }

  public void putTagData() { // publishes tag data to SmartDashboard
    double ttarget = mainTable.getEntry("LimeLight  tv").getDouble(0); // may have to be double then converted
    double tx = mainTable.getEntry("LimeLight tx").getDouble(0);
    double ty = mainTable.getEntry("LimeLight ty").getDouble(0);
    double ta = mainTable.getEntry("LimeLight ta").getDouble(0);
    double tid = mainTable.getEntry("LimeLight tid").getDouble(-1);
    boolean tdetected = ttarget == 0 ? false : true;

    SmartDashboard.putBoolean("LimeLight AprilTag Detected", tdetected);
    SmartDashboard.putNumber("LimeLight Tag X", tx);
    SmartDashboard.putNumber("LimeLight Tag Y", ty);
    SmartDashboard.putNumber("LimeLight Tag Area", ta);
    SmartDashboard.putNumber("LimeLight AprilTag ID", tid);
  }

  public boolean
      noteDetected() { // posts limelight note pipelinedata to SmartDashboard & returns true if note
    // is
    // detected
    double ntarget = mainTable.getEntry("tv").getDouble(0); // may have to be double then converted
    double nx = mainTable.getEntry("tx").getDouble(0);
    double ny = mainTable.getEntry("ty").getDouble(0);
    double na = mainTable.getEntry("ta").getDouble(0);
    boolean ndetected = ntarget == 0 ? false : true;

    SmartDashboard.putBoolean("Note Detected", ndetected);
    SmartDashboard.putNumber("Note X", nx);
    SmartDashboard.putNumber("Note Y", ny);
    SmartDashboard.putNumber("Note Size", na);
    return ndetected;
  }

}