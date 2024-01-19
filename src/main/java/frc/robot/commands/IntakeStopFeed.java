// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopFeed extends InstantCommand {

  private IntakeSubsystem s_IntakeSubsystem;

  public IntakeStopFeed(IntakeSubsystem s_IntakeSubsystem) {
    this.s_IntakeSubsystem = s_IntakeSubsystem;
    addRequirements(this.s_IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_IntakeSubsystem.setMotorSpeed(0);
  }
}

