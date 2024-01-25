// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SampleSubsystem extends SubsystemBase {

  private static CANSparkMax motor = new CANSparkMax(Constants.sampleMotorID, MotorType.kBrushless);
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private static final SparkAbsoluteEncoder m_absEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  private static final int kCPR = 8192;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private double speed = 0;
  private double position = 0;
  /** Creates a new SampleSubsystem. */
  public SampleSubsystem() {

    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kCoast);
    m_pidController = motor.getPIDController();
    m_encoder = motor.getEncoder();
    m_pidController.setFeedbackDevice(m_absEncoder);

    kP = 1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

  }

  public void setMotorSpeed(double speed){
    this.speed = speed;
  }

  public void setPosition(double position){
    this.position = position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //motor.set(speed);
    SmartDashboard.putNumber("Relative Encoder", m_encoder.getPosition());
    SmartDashboard.putNumber("Absolute Encoder", m_absEncoder.getPosition());

    m_pidController.setReference(position, CANSparkMax.ControlType.kPosition);

  }
}
