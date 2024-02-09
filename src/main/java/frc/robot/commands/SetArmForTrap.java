// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SetArmForTrap extends Command {

  private ArmSubsystem a_ArmSubsystem;

  /** Creates a new SetArmForHold. */
  public SetArmForTrap(ArmSubsystem a_ArmSubsystem) {
    this.a_ArmSubsystem = a_ArmSubsystem;
    addRequirements(this.a_ArmSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    a_ArmSubsystem.setRotateSetPoint(Constants.ArmTrapPosition);
    a_ArmSubsystem.setExtendSetPoint(Constants.ExtendTrapPosition);
    a_ArmSubsystem.setWristSetPoint(Constants.WristTrapPosition);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return a_ArmSubsystem.getInPosition();
  }
}

