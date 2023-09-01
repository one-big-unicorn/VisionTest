// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.VisionCommand;

import frc.robot.subsystems.PhotonVision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {

  private final NetworkTableInstance inst;
  private final VisionCommand m_VisionCommand;
  private final PhotonVision m_VisionSubsystem;

  public RobotContainer() {

    inst = NetworkTableInstance.getDefault();
    inst.stopServer();
    inst.setServer(Constants.Constantsq.ROBORIO_IP);
    inst.setServerTeam(695);

    inst.startServer();

    m_VisionSubsystem = new PhotonVision();
    m_VisionCommand = new VisionCommand(m_VisionSubsystem);
    m_VisionSubsystem.setDefaultCommand(m_VisionCommand);
  }

  public Command getAutonomousCommand() {
    return m_VisionCommand;
  }
}

