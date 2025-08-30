package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    Superstructure superstructure;
    Drivetrain drivetrain;

    PhotonCamera tagCamera1 = new PhotonCamera("Tag-01");
    PhotonCamera tagCamera2 = new PhotonCamera("Tag-02");
    PhotonCamera colorCamera1 = new PhotonCamera("Color-01");

    Optional<EstimatedRobotPose> tagCam1VisionEst;
    Optional<EstimatedRobotPose> tagCam2VisionEst;

    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public BooleanSupplier tag1Connected = () -> tagCamera1.isConnected();
    public BooleanSupplier tag2Connected = () -> tagCamera2.isConnected();
    public BooleanSupplier color1Connected = () -> colorCamera1.isConnected();

    PhotonPoseEstimator tag1PhotonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.36, 0.165, 0.11, new Rotation3d(0, 0, 0)));
    PhotonPoseEstimator tag2PhotonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.36, -0.165, 0, new Rotation3d(0, 0, 0)));

    StructPublisher<Pose2d> tag1PosePublisher = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Vision/Tag 1 Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> tag2PosePublisher = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Vision/Tag 2 Pose", Pose2d.struct).publish();

    BooleanPublisher tag1ConnectedPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Subsystems/Vision/Tag 1 Connected").publish();
    BooleanPublisher tag2ConnectedPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Subsystems/Vision/Tag 2 Connected").publish();
    BooleanPublisher color1ConnectedPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Subsystems/Vision/Color 1 Connected").publish();

    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        tag1ConnectedPublisher.set(tag1Connected.getAsBoolean());
        tag2ConnectedPublisher.set(tag2Connected.getAsBoolean());
        color1ConnectedPublisher.set(color1Connected.getAsBoolean());
        if (tag1Connected.getAsBoolean()) {
            var tag1Change = tagCamera1.getLatestResult();
            tagCam1VisionEst = tag1PhotonPoseEstimator.update(tag1Change);
            if (tagCam1VisionEst.isPresent()) {
                drivetrain.addVisionMeasurement(new Pose2d(tagCam1VisionEst.get().estimatedPose.toPose2d().getTranslation(), drivetrain.getState().Pose.getRotation()), tagCam1VisionEst.get().timestampSeconds);
                tag1PosePublisher.set(tagCam1VisionEst.get().estimatedPose.toPose2d());
            }
        }   

        if (tag2Connected.getAsBoolean()) {
            var tag2Change = tagCamera2.getLatestResult();
            tagCam2VisionEst = tag2PhotonPoseEstimator.update(tag2Change);
            if (tagCam2VisionEst.isPresent()) {
                drivetrain.addVisionMeasurement(new Pose2d(tagCam2VisionEst.get().estimatedPose.toPose2d().getTranslation(), drivetrain.getState().Pose.getRotation()), tagCam2VisionEst.get().timestampSeconds);
                tag2PosePublisher.set(tagCam2VisionEst.get().estimatedPose.toPose2d());
            }
        }  
    }

}
