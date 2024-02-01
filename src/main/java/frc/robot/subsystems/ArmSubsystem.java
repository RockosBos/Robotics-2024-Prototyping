// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private static CANSparkMax rotateArm = new CANSparkMax(Constants.ARMROTATEID, MotorType.kBrushless);
  private static CANSparkMax extendArm = new CANSparkMax(Constants.ARMEXTENSIONID, MotorType.kBrushless);
  private static CANSparkMax wristArm = new CANSparkMax(Constants.WRISTID, MotorType.kBrushless);

  private static final SparkAbsoluteEncoder m_rotateAbsEncoder = rotateArm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private static final SparkAbsoluteEncoder m_wristAbsEncoder = wristArm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  private static SparkPIDController rotatePidController;
  private static SparkPIDController extendPidController;
  private static SparkPIDController wristPidController;

  private RelativeEncoder rotateEncoder;
  private RelativeEncoder extendEncoder;
  private RelativeEncoder wristEncoder;

  private double rotateSetPoint = 0;
  private double extendSetPoint = 0;
  private double wristSetPoint = 0;


  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  public ArmSubsystem() {
    rotateArm.restoreFactoryDefaults(true);
    extendArm.restoreFactoryDefaults(true);
    wristArm.restoreFactoryDefaults(true);

      // initialze PID controller and encoder objects

      rotatePidController = rotateArm.getPIDController();
      extendPidController = extendArm.getPIDController();
      wristPidController = wristArm.getPIDController();

      rotatePidController.setFeedbackDevice(m_rotateAbsEncoder);
      wristPidController.setFeedbackDevice(m_wristAbsEncoder);

      rotateEncoder = rotateArm.getEncoder();
      extendEncoder = extendArm.getEncoder();
      wristEncoder = wristArm.getEncoder();
  
      // PID coefficients
      kP = 5e-5; 
      kI = 1e-6;
      kD = 0; 
      kIz = 0; 
      kFF = 0.000156; 
      kMaxOutput = 1; 
      kMinOutput = -1;
      maxRPM = 5700;
  
      // Smart Motion Coefficients
      maxVel = 2000; // rpm
      minVel = 0;
      maxAcc = 1500;
  
      // set PID coefficients

      rotatePidController.setP(kP);
      rotatePidController.setI(kI);
      rotatePidController.setD(kD);
      rotatePidController.setIZone(kIz);
      rotatePidController.setFF(kFF);
      rotatePidController.setOutputRange(kMinOutput, kMaxOutput);

      extendPidController.setP(kP);
      extendPidController.setI(kI);
      extendPidController.setD(kD);
      extendPidController.setIZone(kIz);
      extendPidController.setFF(kFF);
      extendPidController.setOutputRange(kMinOutput, kMaxOutput);
      
      wristPidController.setP(kP);
      wristPidController.setI(kI);
      wristPidController.setD(kD);
      wristPidController.setIZone(kIz);
      wristPidController.setFF(kFF);
      wristPidController.setOutputRange(kMinOutput, kMaxOutput);
      /**
       * Smart Motion coefficients are set on a SparkPIDController object
       * 
       * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
       * the pid controller in Smart Motion mode
       * - setSmartMotionMinOutputVelocity() will put a lower bound in
       * RPM of the pid controller in Smart Motion mode
       * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
       * of the pid controller in Smart Motion mode
       * - setSmartMotionAllowedClosedLoopError() will set the max allowed
       * error for the pid controller in Smart Motion mode
       */
      int rotateSmartMotionSlot = 0;
      int extendSmartMotionSlot = 0;
      int wristSmartMotionSlot = 0;

      rotatePidController.setSmartMotionMaxVelocity(maxVel, rotateSmartMotionSlot);
      rotatePidController.setSmartMotionMinOutputVelocity(minVel, rotateSmartMotionSlot);
      rotatePidController.setSmartMotionMaxAccel(maxAcc, rotateSmartMotionSlot);
      rotatePidController.setSmartMotionAllowedClosedLoopError(allowedErr, rotateSmartMotionSlot);

      extendPidController.setSmartMotionMaxVelocity(maxVel, extendSmartMotionSlot);
      extendPidController.setSmartMotionMinOutputVelocity(minVel, extendSmartMotionSlot);
      extendPidController.setSmartMotionMaxAccel(maxAcc, extendSmartMotionSlot);
      extendPidController.setSmartMotionAllowedClosedLoopError(allowedErr, extendSmartMotionSlot);

      wristPidController.setSmartMotionMaxVelocity(maxVel, wristSmartMotionSlot);
      wristPidController.setSmartMotionMinOutputVelocity(minVel, wristSmartMotionSlot);
      wristPidController.setSmartMotionMaxAccel(maxAcc, wristSmartMotionSlot);
      wristPidController.setSmartMotionAllowedClosedLoopError(allowedErr, wristSmartMotionSlot);
    }

  public void setRotateSetPoint(double rotateSetPoint) {
    this.rotateSetPoint = rotateSetPoint;
  }

  public void setExtendSetPoint(double extendSetPoint) {
    this.extendSetPoint = extendSetPoint;
  }

  public void setWristSetPoint(double wristSetPoint) {
    this.wristSetPoint = wristSetPoint;
  }

  public boolean rotateInPosition() {
    double dif = rotateEncoder.getPosition() - rotateSetPoint;

    if (Math.abs(dif) > Constants.rotateThreshold) {
      return false;
    }
    else {
      return true;
    }
  }

  public boolean extendInPosition() {
    double dif = extendEncoder.getPosition() - extendSetPoint;

    if (Math.abs(dif) > Constants.extendThreshold) {
      return false;
    }
    else {
      return true;
    }
  }
  public boolean wristInPosition() {
    double dif = wristEncoder.getPosition() - wristSetPoint;

    if (Math.abs(dif) > Constants.wristThreshold) {
      return false;
    }
    else {
      return true;
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    
    
    rotatePidController.setReference(rotateSetPoint, CANSparkMax.ControlType.kPosition);
    extendPidController.setReference(extendSetPoint, CANSparkMax.ControlType.kPosition);
    wristPidController.setReference(wristSetPoint, CANSparkMax.ControlType.kPosition);

  }
}
