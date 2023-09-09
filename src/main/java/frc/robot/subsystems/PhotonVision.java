package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;

public class PhotonVision extends SubsystemBase {

    private final NetworkTableInstance inst;
    private final NetworkTable table;

    // BooleanPublisher hasTargets;
    // DoublePublisher time;
    // IntegerPublisher numTargets;

    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d cameraToRobot;
    PhotonPoseEstimator photonPoseEstimator;
    PhotonCamera camera;

    public PhotonVision() {
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("datatable");

        // hasTargets = table.getBooleanTopic("hasTargets").publish();
        // time = table.getDoubleTopic("time").publish();
        // numTargets = table.getIntegerTopic("numTargets").publish();

        // https://github.wpilib.org/allwpilib/docs/beta/java/edu/wpi/first/apriltag/AprilTagFieldLayout.html

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            SmartDashboard.putString("grr", "yay :D");
        } catch (Exception e) {
            SmartDashboard.putString("grr", "grr ._.");

        }

        camera = new PhotonCamera(Constants.Constantsq.CAMERA_NAME);
        cameraToRobot = new Transform3d(new Translation3d(Constants.Constantsq.CAMERA_TO_ROBOT_OFFSET_FORWARD, 0.0,
                Constants.Constantsq.CAMERA_TO_ROBOT_OFFSET_UP), new Rotation3d(0.0, 0.0, 0.0));
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera,
                cameraToRobot);
        SmartDashboard.putString("estimator", photonPoseEstimator.toString());

    }

    public void see() {
        var vision = camera.getLatestResult();

        if (vision.hasTargets()) {
            SmartDashboard.putString("connected0", "connected");
            // hasTargets.set(true);

            // PhotonTrackedTarget target = vision.getBestTarget();
            // Pose3d robotPose =
            // PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
            // aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);

            if (vision.targets.size() == 1) {
                photonPoseEstimator.setPrimaryStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            } else {
                photonPoseEstimator.setPrimaryStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
            }

            SmartDashboard.putString("connected1", "connected");

            EstimatedRobotPose estimatedPose;
            try {
                estimatedPose = photonPoseEstimator.update().get();
                SmartDashboard.putString("grr2", "yay :D");
                SmartDashboard.putString("connected2", "connected");
                Pose3d robotPose = estimatedPose.estimatedPose;
                // time.set(estimatedPose.timestampSeconds);
                List<PhotonTrackedTarget> targetsUsed = estimatedPose.targetsUsed;
                // numTargets.set(targetsUsed.size());

            SmartDashboard.putString("robotPose", robotPose.toString());
            } catch (Exception e) {
                SmartDashboard.putString("grr2", "grr ._.");
            }

        } else {
            // hasTargets.set(false);
        }
    }
}