package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionWrapper {
    private Integer numTargets;
    private Pose3d robotPose;
    private double time;

    public VisionWrapper(Integer inNumTargets, Pose3d inRobotPose, double time) {
        numTargets = inNumTargets;
        robotPose = inRobotPose;
        this.time = time;
    }

    public Integer getTargets() { return numTargets; }

    public Pose3d getRobotPose() { return robotPose; }

    public double getTime() { return time; }

}
