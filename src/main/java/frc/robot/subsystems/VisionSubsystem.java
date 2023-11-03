package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
        cameraToRobot = new Transform3d(new Translation3d(Constants.Vision.ROBOT_TO_CAMERA_OFFSET_FORWARD, Constants.Vision.ROBOT_TO_CAMERA_OFFSET_SIDEWAYS,
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

        Translation3d robotToCube = getRobotToCube();

        if (robotToCube == null) {
            m_field.getObject("cube").setPose(new Pose2d());
            SmartDashboard.getString("cubePose", "none");
            return null;
        }

        Pose2d cubePose = new Pose2d(robotToCube.toTranslation2d().plus(robotPose.getTranslation()), robotPose.getRotation());
        
        m_field.getObject("cube").setPose(cubePose);
        SmartDashboard.getString("cubePose", cubePose.toString());
        return cubePose;
    }

    public Translation3d getRobotToCube() {        
        camera.setPipelineIndex(1);

        //Pose2d robotPose = new Pose2d();
        result = camera.getLatestResult();
        SmartDashboard.putBoolean("Targets?", result.hasTargets());

        if (result.hasTargets()) {
            
            PhotonTrackedTarget target = result.getBestTarget();
            
            Transform3d cameraToTargetTransform = target.getBestCameraToTarget();
            // SmartDashboard.putNumber("Camera To Target Distance", Math.sqrt(Math.pow(cameraToTargetTransform.getX(), 2) + Math.pow(cameraToTargetTransform.getY(), 2) + Math.pow(cameraToTargetTransform.getZ(), 2)));

            Translation3d cameraToTarget = cameraToTargetTransform.getTranslation();

            // SmartDashboard.putString("Camera to Cube", cameraToTarget.toString());
            
            Translation3d robotToCube = cameraToTarget.plus(Constants.Vision.cameraToRobotOffset);
            // SmartDashboard.putString("Robot to Cube", robotToCube.toString());
            

            return robotToCube;

        }
        // SmartDashboard.putString("Camera to Cube", "none");
        // SmartDashboard.putString("Robot to Cube", "none");
        // SmartDashboard.putNumber("Camera To Target Distance", -1);

        return null;
    }

    public Pose3d getRobotPose() {
        camera.setPipelineIndex(0);
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
