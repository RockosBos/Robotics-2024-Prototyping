// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShootingSubsystem;


public class SetShooter extends Command {
    private ShootingSubsystem s_ShootingSubsystem;

  /** Creates a new SetShooter. */
  public SetShooter(ShootingSubsystem s_ShootingSubsystem) {
    this.s_ShootingSubsystem = s_ShootingSubsystem;
    addRequirements(this.s_ShootingSubsystem);



    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_ShootingSubsystem.setMotorVelocity(Constants.LeftMotorVelocity, Constants.RightMotorVelocity);

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
