// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private static CANSparkMax rotateArm;
  private static CANSparkMax extendArm;
  private static CANSparkMax wristArm;

  private static SparkPIDController rotatePidController;
  private static SparkPIDController extendPidController;
  private static SparkPIDController wristPidController;

  private RelativeEncoder rotateEncoder;
  private RelativeEncoder extendEncoder;
  private RelativeEncoder wristEncoder;

  private double rotateSetPoint = 0;
  private double extendSetPoint = 0;
  private double wristSetPoint = 0;

  //carpal tunnel

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  public ArmSubsystem() {
    rotateArm.restoreFactoryDefaults(true);
    extendArm.restoreFactoryDefaults(true);
    wristArm.restoreFactoryDefaults(true);

      // initialze PID controller and encoder objects

      rotatePidController = rotateArm.getPIDController();
      extendPidController = extendArm.getPIDController();
      wristPidController = wristArm.getPIDController();

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



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double p = kP;
    double i = kI;
    double d = kD;
    double iz = kIz;
    double ff = kFF;
    double max = kMaxOutput;
    double min = kMinOutput;
    double maxV = maxVel;
    double minV = minVel;
    double maxA = maxAcc;
    double allE = allowedErr;
    
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    
    
    rotatePidController.setReference(rotateSetPoint, CANSparkMax.ControlType.kSmartMotion);
    extendPidController.setReference(extendSetPoint, CANSparkMax.ControlType.kSmartMotion);
    wristPidController.setReference(wristSetPoint, CANSparkMax.ControlType.kSmartMotion);

  }
}
