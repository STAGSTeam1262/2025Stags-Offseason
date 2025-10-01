package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.Elastic;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.NotificationLevel;

public class Vision extends SubsystemBase {

    Superstructure superstructure;
    Drivetrain drivetrain;

    PhotonCamera tagCamera1 = new PhotonCamera("Tag-01");
    PhotonCamera tagCamera2 = new PhotonCamera("Tag-02");

    Optional<EstimatedRobotPose> tagCam1VisionEst;
    Optional<EstimatedRobotPose> tagCam2VisionEst;

    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public Trigger tag1Connected = new Trigger(() -> tagCamera1.isConnected());
    public Trigger tag2Connected = new Trigger(() -> tagCamera2.isConnected());

    PhotonPoseEstimator tag1PhotonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.43, -0.24, 0.43, new Rotation3d(0, Math.toDegrees(-10), 0)));
    PhotonPoseEstimator tag2PhotonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.43, 0.24, 0.43, new Rotation3d(0, Math.toDegrees(-10), 0)));

    StructPublisher<Pose2d> tag1PosePublisher = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Vision/Tag 1 Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> tag2PosePublisher = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Vision/Tag 2 Pose", Pose2d.struct).publish();

    StructPublisher<Transform3d> tag1TranslationPublisher = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Vision/Tag 1 Translation", Transform3d.struct).publish();
    StructPublisher<Transform3d> tag2TranslationPublisher = NetworkTableInstance.getDefault().getStructTopic("Subsystems/Vision/Tag 2 Translation", Transform3d.struct).publish();

    BooleanPublisher tag1ConnectedPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Subsystems/Vision/Tag 1 Connected").publish();
    BooleanPublisher tag2ConnectedPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Subsystems/Vision/Tag 2 Connected").publish();

    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        tag1TranslationPublisher.set(new Transform3d(0.43, -0.24, 0.37, new Rotation3d(0, Math.toDegrees(-10), 0)));
        tag2TranslationPublisher.set(new Transform3d(0.43, 0.24, 0.43, new Rotation3d(0, Math.toDegrees(-10), 0)));

        tag1Connected
            .onTrue(Commands.runOnce(() -> 
                Elastic.sendNotification(new Notification(NotificationLevel.INFO, "[Vision] Camera Status", "Tag Camera 1 Connected. This camera can send vision updates."))))
            .onFalse(Commands.runOnce(() -> 
                Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "[Vision] Camera Status", "Tag Camera 1 Disconnected. No vision updates will be sent from this camera."))));
        tag2Connected
            .onTrue(Commands.runOnce(() -> 
                Elastic.sendNotification(new Notification(NotificationLevel.INFO, "[Vision] Camera Status", "Tag Camera 2 Connected. This camera can send vision updates."))))
            .onFalse(Commands.runOnce(() -> 
                Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "[Vision] Camera Status", "Tag Camera 2 Disconnected. No vision updates will be sent from this camera."))));
    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        tag1ConnectedPublisher.set(tag1Connected.getAsBoolean());
        tag2ConnectedPublisher.set(tag2Connected.getAsBoolean());

        double omegaRps = Units.radiansToRotations(drivetrain.getState().Speeds.omegaRadiansPerSecond);
        tag1PhotonPoseEstimator.addHeadingData(drivetrain.getState().Timestamp, drivetrain.getState().Pose.getRotation());
        
        if (tag1Connected.getAsBoolean() && Math.abs(omegaRps) < 2.0) {
            var tag1Change = tagCamera1.getLatestResult();
            tagCam1VisionEst = tag1PhotonPoseEstimator.update(tag1Change);
            if (tagCam1VisionEst.isPresent()) {
                drivetrain.addVisionMeasurement(tagCam1VisionEst.get().estimatedPose.toPose2d(), tagCam1VisionEst.get().timestampSeconds);
                tag1PosePublisher.set(tagCam1VisionEst.get().estimatedPose.toPose2d());
            }
        }   
        /*
        if (tag2Connected.getAsBoolean() && Math.abs(omegaRps) < 2.0) {
            var tag2Change = tagCamera2.getLatestResult();
            tagCam2VisionEst = tag2PhotonPoseEstimator.update(tag2Change);
            if (tagCam2VisionEst.isPresent()) {
                drivetrain.addVisionMeasurement(tagCam2VisionEst.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(tagCam2VisionEst.get().timestampSeconds));
                tag2PosePublisher.set(tagCam2VisionEst.get().estimatedPose.toPose2d());
            }
        }
            */
    }

}
