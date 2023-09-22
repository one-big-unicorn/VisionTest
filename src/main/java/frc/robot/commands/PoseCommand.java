// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.VisionSubsystem;

public class PoseCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final VisionSubsystem m_VisionSubsystem;

  public PoseCommand(VisionSubsystem subsystem) {
    m_VisionSubsystem = subsystem;

    addRequirements(m_VisionSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SmartDashboard.putString("output", m_VisionSubsystem.findCube().toString());
    //m_VisionSubsystem.getPose();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}