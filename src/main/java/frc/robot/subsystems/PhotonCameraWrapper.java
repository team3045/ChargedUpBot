
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
        
        final AprilTag tag18 = new AprilTag(18, new Pose3d(new Pose2d(Length, Width / 2.0, Rotation2d.fromDegrees(180))));
        final AprilTag tag01 = new AprilTag(01, new Pose3d(new Pose2d(0.0, Width / 2.0, Rotation2d.fromDegrees(0.0))));

        ArrayList<AprilTag> atlList = new ArrayList<AprilTag>();
        atlList.add(tag18);
        atlList.add(tag01);


        AprilTagFieldLayout atfl = new AprilTagFieldLayout(atlList, Length, Width);

        photoncamera = new PhotonCamera(CameraName);
        photonPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photoncamera, null);
    }

    public Optional<EstimatedRobotPose> getEstimedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

}