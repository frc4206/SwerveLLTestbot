// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class ChangePipelineCommand extends CommandBase {
  /** Creates a new ChangePipelineCommand. */
  Limelight m_Limelight;
  int m_pip;
  public ChangePipelineCommand(Limelight Limelight, int pip) {
    m_Limelight = Limelight;
    m_pip = pip;
    addRequirements(Limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(m_pip);
    m_Limelight.ChangePipelines(m_pip);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
