// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbingSubsystem;
  
public class SetClimbLow extends Command {
 
  private ClimbingSubsystem c_ClimbingSubsysetm;
 
  /** Creates a new SetClimbLow. */
  public SetClimbLow(ClimbingSubsystem c_ClimbingSubsysetm) {
    this.c_ClimbingSubsysetm = c_ClimbingSubsysetm;
    addRequirements(this.c_ClimbingSubsysetm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c_ClimbingSubsysetm.leftClimbRotateSetPoint(Constants.leftClimbLow);
    c_ClimbingSubsysetm.rightClimbExtendSetPoint(Constants.rightClimbLow);
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
