// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Vision {
    public static final int kDriverControllerPort = 0;
    public static final double CAMERA_HEIGHT_METERS = 0.0;
    public static final double CAMERA_TO_ROBOT_OFFSET_FORWARD = 0.0;
    public static final double CAMERA_TO_ROBOT_OFFSET_SIDEWAYS = 0.0;
    public static final String CAMERA_NAME = "USB_2.0_1080P_Camera";
    public static final String ROBORIO_IP = "[10.6.95.12]";
  }
}
