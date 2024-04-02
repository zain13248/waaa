// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class limelight {

    String name;
    NetworkTable table;

    public limelight(String name){
        this.name = name;
        this.table = NetworkTableInstance.getDefault().getTable(name);
    }

    public double getLimelightTX(){
        return this.table.getEntry("tx").getDouble(0);
    }
    public double getLimelightTY(){
        return this.table.getEntry("ty").getDouble(0);
    }
    public double getLimelightTA(){
        return this.table.getEntry("ta").getDouble(0);
    }
    public void lightsOff(){
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("ledMode").setNumber(1);
    }
    public void lightsOn(){
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("ledMode").setNumber(0);
    }
    public void lightsForceOn(){
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("ledMode").setNumber(3);
    }

        public void lightsForceOnBlink(){
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("ledMode").setNumber(2);
    }
    public boolean hasTarget(){
        double targetAmount=this.table.getEntry("tv").getDouble(0);
        if (targetAmount>0){
            return true;
        }else return false;
    }


    public Pose2d getPose(){
        return null;
    }
}