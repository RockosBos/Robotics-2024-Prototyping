// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {

  PhotonCamera camera;
  PhotonTrackedTarget target;
  double yaw = 0.0;
  double pitch = 0.0;
  PhotonPipelineResult result;
  boolean hasTargets = false;

  PIDController rotatePIDController;

  /** Creates a new PhotonVision. */
  public PhotonVision() {
    camera = new PhotonCamera("C270_HD_WEBCAM");
    rotatePIDController = new PIDController(0.01, 0, 0);
    
  }

  public boolean getHasTarget() {
    return hasTargets;
  }

  public double getYaw() {
    return yaw;
  }

  public double getPIDRotateValue(){
    return rotatePIDController.calculate(yaw, 0.0);
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();
    hasTargets = result.hasTargets();
    target = result.getBestTarget();
    yaw = target.getYaw();
    pitch = target.getPitch();

    SmartDashboard.putBoolean("hasTarget", hasTargets);
    SmartDashboard.putNumber("yaw", yaw);
  }
}
