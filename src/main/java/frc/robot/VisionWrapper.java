package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionWrapper {
    private int numTargets;
    private Pose3d robotPose;
    private double time;

    public VisionWrapper(int inNumTargets, Pose3d inRobotPose, double time) {
        numTargets = inNumTargets;
        robotPose = inRobotPose;
        this.time = time;
    }

    public int getTargets() { return numTargets; }

    public Pose3d getRobotPose() { return robotPose; }

    public double getTime() { return time; }

}
