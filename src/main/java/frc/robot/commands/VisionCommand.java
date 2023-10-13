// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final VisionSubsystem m_VisionSubsystem;

  public VisionCommand(VisionSubsystem subsystem) {
    m_VisionSubsystem = subsystem;

    addRequirements(m_VisionSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_VisionSubsystem.getRobotToCube();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}