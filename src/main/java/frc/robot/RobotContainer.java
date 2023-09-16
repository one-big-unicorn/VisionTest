// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CubeCommand;
import frc.robot.commands.PoseCommand;
import frc.robot.subsystems.VisionSubsystem;

// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  // private final NetworkTableInstance inst;
  private final PoseCommand m_PoseCommand;
  private final VisionSubsystem m_VisionSubsystem;
  private final CubeCommand m_CubeCommand;

  public RobotContainer() {

    // inst = NetworkTableInstance.getDefault();
    // inst.stopServer();
    // inst.setServerTeam(695);

    // inst.startServer();

    m_VisionSubsystem = new VisionSubsystem();
    m_PoseCommand = new PoseCommand(m_VisionSubsystem);
    m_CubeCommand = new CubeCommand(m_VisionSubsystem);

    m_VisionSubsystem.setDefaultCommand(m_PoseCommand);

  }

  public Command getAutonomousCommand() {
    return m_PoseCommand;
  }
}
