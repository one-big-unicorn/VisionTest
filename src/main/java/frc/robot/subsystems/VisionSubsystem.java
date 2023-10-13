package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera camera;
    PhotonPipelineResult result;
    Field2d m_field = new Field2d();
    PhotonPoseEstimator photonPoseEstimator;
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d cameraToRobot;

    public VisionSubsystem() {
        camera = new PhotonCamera(Constants.Vision.CAMERA_NAME);
        camera.setLED(VisionLEDMode.kOff);
        SmartDashboard.putData("Field", m_field);
        cameraToRobot = new Transform3d(new Translation3d(Constants.Vision.CAMERA_TO_ROBOT_OFFSET_FORWARD, Constants.Vision.CAMERA_TO_ROBOT_OFFSET_SIDEWAYS,
                Constants.Vision.CAMERA_HEIGHT_METERS), new Rotation3d(0.0, 0.0, 0.0));
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
            SmartDashboard.putString("field layout load error", e.toString());
        }

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, camera,
                cameraToRobot);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    public Pose2d getFieldtoCube(Pose2d robotPose) {

        if (robotPose == null) {
            m_field.getObject("cube").setPose(new Pose2d());
            SmartDashboard.getString("cubePose", "none");
            return null;
        }

        Pose2d robotToCube = getRobotToCube();

        if (robotToCube == null) {
            m_field.getObject("cube").setPose(new Pose2d());
            SmartDashboard.getString("cubePose", "none");
            return null;
        }

        Pose2d cubePose = new Pose2d(robotToCube.getTranslation().plus(robotPose.getTranslation()), robotPose.getRotation());
        
        m_field.getObject("cube").setPose(cubePose);
        SmartDashboard.getString("cubePose", cubePose.toString());
        return cubePose;
    }

    public Pose2d getRobotToCube() {

        camera.setPipelineIndex(0);
        result = camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            double yaw = target.getYaw();

            Pose2d cubePose = new Pose2d(
                PhotonUtils.estimateCameraToTargetTranslation(
                    PhotonUtils.calculateDistanceToTargetMeters(
                        Constants.Vision.CAMERA_HEIGHT_METERS, 0, 0, target.getPitch()),
                    Rotation2d.fromDegrees(yaw * -1)
                ),
                new Rotation2d(yaw)
            );

            return cubePose;
        }

        return null;
    }

    public Pose3d getRobotPose() {
        camera.setPipelineIndex(1);
        PhotonPipelineResult tresult;

        EstimatedRobotPose estimatedPose;
        Pose3d robotPose;

        tresult = camera.getLatestResult();

        if (tresult.hasTargets()) {
            SmartDashboard.putBoolean("targets?", true);

            var thingy = photonPoseEstimator.update(tresult);
            try {
                estimatedPose = thingy.get();
                robotPose = estimatedPose.estimatedPose;
            } catch (Exception e) {
                estimatedPose = null;
                robotPose = null;
                return null;
            }

            try {
                m_field.setRobotPose(robotPose.toPose2d());

                SmartDashboard.putString("robotPose", robotPose.toString());

                return robotPose;

            } catch (Exception e) {
                SmartDashboard.putString("robotPose", "none");
                m_field.setRobotPose(new Pose2d());
            }
        } else {
            SmartDashboard.putBoolean("targets?", false);
            SmartDashboard.putString("robotPose", "none");
            m_field.setRobotPose(new Pose2d());
        }

        return null;
    }
}
