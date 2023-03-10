
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonCameraWrapper extends SubsystemBase {
    public PhotonCamera photoncamera;
    public PhotonPoseEstimator photonPoseEstimator;
    final Integer Length = 10;
    final Integer Width = 5;
    final String CameraName = ("PoseEstimate");
    



    public PhotonCameraWrapper() {
        
        
        final AprilTag tag01 = new AprilTag(01, new Pose3d(new Pose2d(37.5, 1 , Rotation2d.fromDegrees(0.0))));
        final AprilTag tag2 = new AprilTag(2, new Pose3d(new Pose2d(108.25, 1 , Rotation2d.fromDegrees(0))));
        final AprilTag tag3 = new AprilTag(3, new Pose3d(new Pose2d(178.5, 1, Rotation2d.fromDegrees(0))));
        final AprilTag tag4 = new AprilTag(4, new Pose3d(new Pose2d(267.5, -4, Rotation2d.fromDegrees(0))));
        final AprilTag tag5 = new AprilTag(5, new Pose3d(new Pose2d(267.5, 655.25, Rotation2d.fromDegrees(180))));
        final AprilTag tag6 = new AprilTag(6, new Pose3d(new Pose2d(178.5, 651.25, Rotation2d.fromDegrees(180))));
        final AprilTag tag7 = new AprilTag(7, new Pose3d(new Pose2d(108.25, 651.25, Rotation2d.fromDegrees(180))));
        final AprilTag tag8 = new AprilTag(8, new Pose3d(new Pose2d(37.5, 651.25, Rotation2d.fromDegrees(180))));


        ArrayList<AprilTag> atlList = new ArrayList<AprilTag>();
        atlList.add(tag2);
        atlList.add(tag01);
        atlList.add(tag3);
        atlList.add(tag4);
        atlList.add(tag5);
        atlList.add(tag6);
        atlList.add(tag7);
        atlList.add(tag8);


        AprilTagFieldLayout atfl = new AprilTagFieldLayout(atlList, Length, Width);

        photoncamera = new PhotonCamera(CameraName);
        photonPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photoncamera, null);
    }

    public Optional<EstimatedRobotPose> getEstimedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

}