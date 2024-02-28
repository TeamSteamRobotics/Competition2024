// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.google.gson.Gson;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilVisionSubsystem extends SubsystemBase {
    Coordinate coordinate = new Coordinate();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tableEntry = table.getEntry("json");
    int fidLocation;
    boolean fidLocFound;
    public AprilVisionSubsystem() {}
    Gson gson = new Gson();
    public enum ReturnTarget{
        TARGET,
        ROBOT,
        FIELD
    }
    public Coordinate getCoordinates(int targetId, ReturnTarget rt) {
       switch(rt){
        case TARGET:
        updateTargetCoordinates(targetId);
        break;
        case ROBOT:
        updateRobotCoordinates(targetId);
        break;
        case FIELD:
        updateFieldCoordinates(targetId);
        break;
       }
        return coordinate;
    }

    private void updateTargetCoordinates(int targetId) {
        String jsonString = tableEntry.getString("");
        limelightjson thirteenthReason = gson.fromJson(jsonString, limelightjson.class);
        if (thirteenthReason.Results.Fiducial.length != 0) {
             for (int i = 0; i < thirteenthReason.Results.Fiducial.length; i++) {
                if(thirteenthReason.Results.Fiducial[i].fID == targetId){
                    fidLocation = i;
                    fidLocFound = true;
                    break;
                }else{
                    fidLocFound = false;
                }
            }if(!fidLocFound){
                //System.out.println("TARGET FIDUCIAL NOT FOUND!");
                coordinate.aprilTagVisible = false;
            }else{
                coordinate.x = thirteenthReason.Results.Fiducial[fidLocation].t6t_rs[0];
                coordinate.y = thirteenthReason.Results.Fiducial[fidLocation].t6t_rs[1];
                coordinate.z = thirteenthReason.Results.Fiducial[fidLocation].t6t_rs[2];
                coordinate.rx = thirteenthReason.Results.Fiducial[fidLocation].t6t_rs[3];
                coordinate.ry = thirteenthReason.Results.Fiducial[fidLocation].t6t_rs[4];
                coordinate.rz = thirteenthReason.Results.Fiducial[fidLocation].t6t_rs[5];
                coordinate.aprilTagVisible = true;
            } 
        }else{
            coordinate.aprilTagVisible = false;
            System.out.println("NO FIDUCIALS IN VIEW!");
        }
    }
    private void updateRobotCoordinates(int targetId) {
        String jsonString = tableEntry.getString("");
        limelightjson thirteenthReason = gson.fromJson(jsonString, limelightjson.class);
        if (thirteenthReason.Results.Fiducial.length != 0) {
             for (int i = 0; i < thirteenthReason.Results.Fiducial.length; i++) {
                if(thirteenthReason.Results.Fiducial[i].fID == targetId){
                    fidLocation = i;
                    fidLocFound = true;
                    break;
                }else{
                    fidLocFound = false;
                }
            }if(!fidLocFound){
                System.out.println("TARGET FIDUCIAL NOT FOUND!");
                coordinate.aprilTagVisible = false;
            }else{
                coordinate.x = thirteenthReason.Results.Fiducial[fidLocation].t6r_ts[0];
                coordinate.y = thirteenthReason.Results.Fiducial[fidLocation].t6r_ts[1];
                coordinate.z = thirteenthReason.Results.Fiducial[fidLocation].t6r_ts[2];
                coordinate.rx = thirteenthReason.Results.Fiducial[fidLocation].t6r_ts[3];
                coordinate.ry = thirteenthReason.Results.Fiducial[fidLocation].t6r_ts[4];
                coordinate.rz = thirteenthReason.Results.Fiducial[fidLocation].t6r_ts[5];
                coordinate.aprilTagVisible = true;
            } 
        }else{
            coordinate.aprilTagVisible = false;
            System.out.println("NO FIDUCIALS IN VIEW!");
        }
    }
    private void updateFieldCoordinates(int targetId) {
        String jsonString = tableEntry.getString("");
        limelightjson thirteenthReason = gson.fromJson(jsonString, limelightjson.class);
        if (thirteenthReason.Results.Fiducial.length != 0) {
                coordinate.x = thirteenthReason.Results.Fiducial[0].t6r_fs[0];
                coordinate.y = thirteenthReason.Results.Fiducial[0].t6r_fs[1];
                coordinate.z = thirteenthReason.Results.Fiducial[0].t6r_fs[2];
                coordinate.rx = thirteenthReason.Results.Fiducial[0].t6r_fs[3];
                coordinate.ry = thirteenthReason.Results.Fiducial[0].t6r_fs[4];
                coordinate.rz = thirteenthReason.Results.Fiducial[0].t6r_fs[5];
                coordinate.aprilTagVisible = true;
        }else{
            coordinate.aprilTagVisible = false;
            System.out.println("NO FIDUCIALS IN VIEW!");
        }
    }
public class Coordinate {
    public double x;
    public double y;
    public double z;
    public double rx;
    public double ry;
    public double rz;
    public boolean aprilTagVisible;
}

class limelightjson{
    public ResultJson Results;
}

class ResultJson
{
    public FiducialJson[] Fiducial;
    public int pID;
    public double tl;
    public double ts;
    public int v;
}

class FiducialJson
{
    public int fID;
    public String fam;
    public double[] t6t_rs;
    public double[] t6r_ts;
    public double[] t6r_fs;
}
}