// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SampleSubsystem;

public class SetSampleMotor extends Command {

  private SampleSubsystem s_SampleSubsystem;
  /** Creates a new SetSampleMotor. */
  public SetSampleMotor(SampleSubsystem s_SampleSubsystem) {
    this.s_SampleSubsystem = s_SampleSubsystem;
    addRequirements(this.s_SampleSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_SampleSubsystem.setMotorSpeed(Constants.SampleSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
