// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private static CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKEID, MotorType.kBrushless);
  private static RelativeEncoder encoder;

  private double speed = 0;
  /** Creates a new SampleSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.restoreFactoryDefaults(true);
    encoder = intakeMotor.getEncoder();
  }

  public void setMotorSpeed(double speed){
    this.speed = speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    intakeMotor.set(speed);
    
  }
}
