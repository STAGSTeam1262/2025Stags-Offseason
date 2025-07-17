package frc.robot.subsystems;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.utils.Elastic;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.NotificationLevel;

public class Superstructure extends SubsystemBase {

    /***
     * The different states of the robot.
     */
    public enum RobotState {
        IDLE("kIdle"),
        CORAL("kCoral"),
        ALGAE("kAlgae"),
        CLIMB("kClimb");

        private String displayName;

        RobotState(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    };

    public RobotState robotState = RobotState.IDLE;

    public Drivetrain drivetrain;

    // NTPublishers
    StringPublisher robotStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Superstructure/RobotState").publish();

    // Triggers
    public Trigger isEnabled = new Trigger(() -> DriverStation.isEnabled());
    public Trigger isTeleop = new Trigger(() -> DriverStation.isTeleop());
    public Trigger isTeleopEnabled = isEnabled.and(isTeleop);
    public Trigger isAuto = new Trigger(() -> DriverStation.isAutonomous());
    public Trigger isAutoEnabled = isEnabled.and(isAuto);
    public Trigger isEndgame = isTeleopEnabled.and(() -> DriverStation.getMatchTime() != -1 && DriverStation.getMatchTime() <= 20);
    
    public Trigger isIdleState = new Trigger(() -> robotState == RobotState.IDLE);
    public Trigger isCoralScoringState = new Trigger(() -> robotState == RobotState.CORAL);
    public Trigger isAlgaeScoringState = new Trigger(() -> robotState == RobotState.ALGAE);
    public Trigger isClimbState = new Trigger(() -> robotState == RobotState.CLIMB);
    
    
    /*** Tells each subsystem what it's task is currently/how to respond to it's own wanted states. */
    public Superstructure(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        setupTriggers();
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public void setupTriggers() {
        isTeleopEnabled.onTrue(setRobotState(RobotState.IDLE));
        isEndgame.onTrue(
            Constants.OperatorConstants.driverController.blinkRumble(0.5, RumbleType.kBothRumble, 0.2)
            .alongWith(Constants.OperatorConstants.operatorController.blinkRumble(0.5, RumbleType.kBothRumble, 0.2)));
    }

    /*** Sets the overall scoring mode of the robot.
     * @param state The state to set.
     * @return Command to run that sets state.
     */
    public void setState(RobotState state) {
        this.robotState = state;
    }

    /*** Sets the overall scoring mode of the robot.
     * @param state The state to set.
     * @return Command to run that sets state.
     */
    public Command setRobotState(RobotState state) {
        return Commands.runOnce(() -> setState(state));
    }

    public void handleState() {
        // Empty until we decide what needs to go here.
    }

    /*** Acts as a fake disable, running stop method in all other subsystems. */
    public void stop() {

    }

    public Pose2d getRobotPose() {
        return drivetrain.getState().Pose;
    }

    public APTarget getAlignmentTarget() {
        return drivetrain.target;
    }

    public void notifyInfo(String title, String desc, double secondsDisplayed) {
        Notification info = new Notification(NotificationLevel.INFO, title, desc).withDisplaySeconds(secondsDisplayed);
        Elastic.sendNotification(info);
    }

    public void notifyWarning(String title, String desc, double secondsDisplayed) {
        Notification warn = new Notification(NotificationLevel.INFO, title, desc).withDisplaySeconds(secondsDisplayed);
        Elastic.sendNotification(warn);
    }

    public void notifyError(String title, String desc, double secondsDisplayed) {
        Notification error = new Notification(NotificationLevel.ERROR, title, desc).withDisplaySeconds(secondsDisplayed);
        Elastic.sendNotification(error);
    }

    @Override
    public void periodic() {
        robotStatePublisher.set(robotState.getDisplayName());
    }

}
