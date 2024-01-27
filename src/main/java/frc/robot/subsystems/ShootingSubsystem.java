// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootingSubsystem extends SubsystemBase {
  /** Creates a new ShootingSubsystem. */

  private static CANSparkMax ShootingMotorLeft = new CANSparkMax(Constants.LEFTSHOOTERID, MotorType.kBrushless);
  private static CANSparkMax ShootingMotorRight = new CANSparkMax(Constants.RIGHTSHOOTERID, MotorType.kBrushless);

  private SparkPIDController m_leftpidController;
  private SparkPIDController m_rightpidController;

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  

  private double speed = 0;
  private double leftVelocity = 0.0;
  private double rightVelocity = 0.0;
  
  public double leftkP, leftkI, leftkD, leftkIz, leftkFF, leftkMaxOutput, leftkMinOutput, leftmaxRPM, leftmaxVel, leftminVel, leftmaxAcc, leftallowedErr;
  public double rightkP, rightkI, rightkD, rightkIz, rightkFF, rightkMaxOutput, rightkMinOutput, rightmaxRPM, rightmaxVel, rightminVel, rightmaxAcc, rightallowedErr;


  public ShootingSubsystem(){

    ShootingMotorLeft.restoreFactoryDefaults();
    ShootingMotorRight.restoreFactoryDefaults();

    m_leftpidController = ShootingMotorLeft.getPIDController();
    m_rightpidController = ShootingMotorRight.getPIDController();

    m_leftEncoder = ShootingMotorLeft.getEncoder();
    m_rightEncoder = ShootingMotorRight.getEncoder();
    
    // PID coefficients
     leftkP = 5e-5; 
     leftkI = 1e-6;
     leftkD = 0; 
     leftkIz = 0; 
     leftkFF = 0.000156; 
     leftkMaxOutput = 1; 
     leftkMinOutput = -1;
     leftmaxRPM = 5700;
 
     // Smart Motion Coefficients
     leftmaxVel = 5700; // rpm
     leftminVel = 0;
     leftmaxAcc = 5850;
 
     // set PID coefficients

     m_leftpidController.setP(leftkP);
     m_leftpidController.setI(leftkI);
     m_leftpidController.setD(leftkD);
     m_leftpidController.setIZone(leftkIz);
     m_leftpidController.setFF(leftkFF);
     m_leftpidController.setOutputRange(leftkMinOutput, leftkMaxOutput);

     // PID coefficients
     rightkP = 5e-5; 
     rightkI = 1e-6;
     rightkD = 0; 
     rightkIz = 0; 
     rightkFF = 0.000156; 
     rightkMaxOutput = 1; 
     rightkMinOutput = -1;
     rightmaxRPM = 5700;
 
     // Smart Motion Coefficients
     rightmaxVel = 5700; // rpm
     rightminVel = 0;
     rightmaxAcc = 5850;
 
     // set PID coefficients

     m_rightpidController.setP(rightkP);
     m_rightpidController.setI(rightkI);
     m_rightpidController.setD(rightkD);
     m_rightpidController.setIZone(rightkIz);
     m_rightpidController.setFF(rightkFF);
     m_rightpidController.setOutputRange(rightkMinOutput, rightkMaxOutput);

     int leftSmartMotionSlot = 0;
     int rightSmartMotionSlot = 0;

     m_leftpidController.setSmartMotionMaxVelocity(leftmaxVel, leftSmartMotionSlot);
     m_leftpidController.setSmartMotionMinOutputVelocity(leftminVel, leftSmartMotionSlot);
     m_leftpidController.setSmartMotionMaxAccel(leftmaxAcc, leftSmartMotionSlot);
     m_leftpidController.setSmartMotionAllowedClosedLoopError(leftallowedErr, leftSmartMotionSlot);

     m_rightpidController.setSmartMotionMaxVelocity(rightmaxVel, rightSmartMotionSlot);
     m_rightpidController.setSmartMotionMinOutputVelocity(rightminVel, rightSmartMotionSlot);
     m_rightpidController.setSmartMotionMaxAccel(rightmaxAcc, rightSmartMotionSlot);
     m_rightpidController.setSmartMotionAllowedClosedLoopError(rightallowedErr, rightSmartMotionSlot);
  }


  public void setMotorVelocity(double leftVelocity, double rightVelocity){
    this.leftVelocity = leftVelocity;
    this.rightVelocity = rightVelocity;
  }
 
  public void setMotorSpeed(double speed){
    this.speed = speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Set Velocity", leftVelocity);
    SmartDashboard.putNumber("Right Set Velocity", rightVelocity);
    SmartDashboard.putNumber("Left Velocity", ShootingMotorLeft.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Velocity", ShootingMotorRight.getEncoder().getVelocity());
   m_leftpidController.setReference(leftVelocity, CANSparkMax.ControlType.kSmartVelocity);
   m_rightpidController.setReference(rightVelocity, CANSparkMax.ControlType.kSmartVelocity);
  }
}
