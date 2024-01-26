// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  double[] poseData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
  double poseX, poseY, poseZ, poseRoll, posePitch, poseYaw;
  double tv;
  Alliance alliance = Alliance.Blue;
  String allianceString = "";

  /** Creates a new Limelight. */
  public Limelight() {
    
  }

  public double[] getLLPose(){
    return poseData;
  }

  public boolean validTarget(){
    if(tv == 1){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    alliance = DriverStation.getAlliance().get();

    if(alliance == Alliance.Blue){
      allianceString = "botpose_wpiblue";
    }
    else if(alliance == Alliance.Red){
      allianceString = "botpose_wpired";
    }
    else{
      allianceString = "botpose";
    }

    tv = table.getEntry("tv").getDouble(0.0);
    
    poseData = NetworkTableInstance.getDefault().getTable("limelight").getEntry(allianceString).getDoubleArray(new double[6]);
    poseX = poseData[0];
    poseY = poseData[1];
    poseZ = poseData[2];
    poseRoll = poseData[3];
    posePitch = poseData[4];
    poseYaw = poseData[5];

    SmartDashboard.putNumber("Limelight PoseX", poseX);
    SmartDashboard.putNumber("Limelight PoseY", poseY);
    SmartDashboard.putNumber("Limelight PoseYaw", poseYaw);
    SmartDashboard.putNumber("Limelight Valid Target?", tv);

  }
}
