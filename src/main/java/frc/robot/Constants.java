// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;

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
    // Offsets are from ROBOT to CAMERA, so if offset of 0.2, 0.2, 0.2, that means camera is to the upwards top left of robot
    public static final double CAMERA_HEIGHT_METERS = 0.2;
    public static final double ROBOT_TO_CAMERA_OFFSET_FORWARD = 0.2;
    public static final double ROBOT_TO_CAMERA_OFFSET_SIDEWAYS = 0.2;

    public static final String CAMERA_NAME = "USB_2.0_1080P_Camera";
    public static final double INTAKE_OFFSET = 0.3;
    public static final Translation3d cameraToRobotOffset = new Translation3d(Constants.Vision.ROBOT_TO_CAMERA_OFFSET_FORWARD,
            Constants.Vision.ROBOT_TO_CAMERA_OFFSET_SIDEWAYS,
            Constants.Vision.CAMERA_HEIGHT_METERS);
  }
}
