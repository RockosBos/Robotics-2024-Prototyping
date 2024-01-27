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

public class ClimbingSubsystem extends SubsystemBase {
    private static CANSparkMax leftClimb = new CANSparkMax(Constants.LEFTCLIMBID, MotorType.kBrushless);
    private static CANSparkMax rightClimb = new CANSparkMax(Constants.RIGHTCLIMBID, MotorType.kBrushless);
    
    private static SparkPIDController leftClimbPidController;
    private static SparkPIDController rightClimbPidController;

    private RelativeEncoder leftClimbEncoder;
    private RelativeEncoder rightClimbEncoder;

    private double leftClimbSetPoint = 0;
    private double rightClimbSetPoint = 0;
    
    public double leftkP, leftkI, leftkD, leftkIz, leftkFF, leftkMaxOutput, leftkMinOutput, leftmaxRPM, leftmaxVel, leftminVel, leftmaxAcc, leftallowedErr;
  public double rightkP, rightkI, rightkD, rightkIz, rightkFF, rightkMaxOutput, rightkMinOutput, rightmaxRPM, rightmaxVel, rightminVel, rightmaxAcc, rightallowedErr;
  /** Creates a new ClimbingSubsystem. */
  public ClimbingSubsystem() {
    leftClimb.restoreFactoryDefaults(true);
    rightClimb.restoreFactoryDefaults(true);

    leftClimbPidController = leftClimb.getPIDController();
    rightClimbPidController = rightClimb.getPIDController();

    leftClimbEncoder = leftClimb.getEncoder();
    rightClimbEncoder = rightClimb.getEncoder();

    leftkP = 5e-5; 
    leftkP = 5e-5; 
    leftkI = 1e-6;
    leftkD = 0; 
    leftkIz = 0; 
    leftkFF = 0.000156; 
    leftkMaxOutput = 1; 
    leftkMinOutput = -1;
    leftmaxRPM = 5700;

    leftmaxVel = 5700; // rpm
    leftminVel = 0;
    leftmaxAcc = 5850;

      leftClimbPidController.setP(leftkP);
      leftClimbPidController.setI(leftkI);
      leftClimbPidController.setD(leftkD);
      leftClimbPidController.setIZone(leftkIz);
      leftClimbPidController.setFF(leftkFF);
      leftClimbPidController.setOutputRange(leftkMinOutput, leftkMaxOutput);

      rightkP = 5e-5; 
      rightkI = 1e-6;
      rightkD = 0; 
      rightkIz = 0; 
      rightkFF = 0.000156; 
      rightkMaxOutput = 1; 
      rightkMinOutput = -1;
      rightmaxRPM = 5700;

      rightmaxVel = 5700; // rpm
      rightminVel = 0;
      rightmaxAcc = 5850;
 

      rightClimbPidController.setP(rightkP);
      rightClimbPidController.setI(rightkI);
      rightClimbPidController.setD(rightkD);
      rightClimbPidController.setIZone(rightkIz);
      rightClimbPidController.setFF(rightkFF);
      rightClimbPidController.setOutputRange(rightkMinOutput, rightkMaxOutput);

      int leftClimbSmartMotionSlot = 0;
      int rightClimbSmartMotionSlot = 0;
      
      leftClimbPidController.setSmartMotionMaxVelocity(leftmaxVel, leftClimbSmartMotionSlot);
      leftClimbPidController.setSmartMotionMinOutputVelocity(leftminVel, leftClimbSmartMotionSlot);
      leftClimbPidController.setSmartMotionMaxAccel(leftmaxAcc, leftClimbSmartMotionSlot);
      leftClimbPidController.setSmartMotionAllowedClosedLoopError(leftallowedErr, leftClimbSmartMotionSlot);

      rightClimbPidController.setSmartMotionMaxVelocity(rightmaxVel, rightClimbSmartMotionSlot);
      rightClimbPidController.setSmartMotionMinOutputVelocity(rightminVel, rightClimbSmartMotionSlot);
      rightClimbPidController.setSmartMotionMaxAccel(rightmaxAcc, rightClimbSmartMotionSlot);
      rightClimbPidController.setSmartMotionAllowedClosedLoopError(rightallowedErr, rightClimbSmartMotionSlot);
  }

  public void leftClimbRotateSetPoint(double leftClimbSetPoint) {
    this.leftClimbSetPoint = leftClimbSetPoint;
  }

  public void rightClimbExtendSetPoint(double rightClimbSetPoint) {
    this.rightClimbSetPoint = rightClimbSetPoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    leftClimbPidController.setReference(leftClimbSetPoint, CANSparkMax.ControlType.kSmartMotion);
    rightClimbPidController.setReference(rightClimbSetPoint, CANSparkMax.ControlType.kSmartMotion);
  }
}
