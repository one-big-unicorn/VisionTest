// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.VisionCommand;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  private final VisionSubsystem m_VisionSubsystem;
  private final VisionCommand m_PoseCommand;

  public RobotContainer() {

    m_VisionSubsystem = new VisionSubsystem();
    m_PoseCommand = new VisionCommand(m_VisionSubsystem);
    m_VisionSubsystem.setDefaultCommand(m_PoseCommand);

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
