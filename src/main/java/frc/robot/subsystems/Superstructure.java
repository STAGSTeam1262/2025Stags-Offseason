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

public class Superstructure extends SubsystemBase {

    public enum RobotState {
        DISABLED("kDisabled"),
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

    public RobotState robotState = RobotState.DISABLED;

    CommandSwerveDrivetrain drivetrain;

    // NTPublishers
    StringPublisher robotStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Superstructure/RobotState").publish();

    // Triggers
    public Trigger isEnabled = new Trigger(() -> DriverStation.isEnabled());
    public Trigger isTeleop = new Trigger(() -> DriverStation.isTeleop());
    public Trigger isTeleopEnabled = isEnabled.and(isTeleop);
    public Trigger isAuto = new Trigger(() -> DriverStation.isAutonomous());
    public Trigger isAutoEnabled = isEnabled.and(isAuto);
    public Trigger isEndgame = isTeleopEnabled.and(() -> DriverStation.getMatchTime() != -1 && DriverStation.getMatchTime() <= 20);
    
    
    /*** Tells each subsystem what it's task is currently/how to respond to it's own wanted states. */
    public Superstructure(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        setupTriggers();
    }

    public void setupTriggers() {
        isTeleopEnabled.onTrue(setRobotState(RobotState.IDLE));
        isEndgame.onTrue(
            Constants.OperatorConstants.driverController.blinkRumble(0.5, RumbleType.kBothRumble, 0.2)
            .alongWith(Constants.OperatorConstants.operatorController.blinkRumble(0.5, RumbleType.kBothRumble, 0.2)));
    }

    public void setState(RobotState state) {
        this.robotState = state;
    }

    public Command setRobotState(RobotState state) {
        return Commands.runOnce(() -> setState(state));
    }

    public void handleState() {

    }

    public Pose2d getRobotPose() {
        return drivetrain.getState().Pose;
    }

    public APTarget getAlignmentTarget() {
        return drivetrain.target;
    }

    @Override
    public void periodic() {
        robotStatePublisher.set(robotState.getDisplayName());
    }

}
