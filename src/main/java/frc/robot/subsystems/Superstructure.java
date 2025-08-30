package frc.robot.subsystems;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.State;
import frc.robot.utils.Elastic;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.NotificationLevel;

public class Superstructure extends SubsystemBase {

    /***
     * The different states of the robot.
     */
    public enum RobotMode {
        IDLE("kIdle", new Color(0, 0, 0)),
        CORAL("kCoral", new Color(200, 200, 200)),
        ALGAE("kAlgae", new Color(28, 254, 112)),
        CLIMB("kClimb", new Color(255, 0, 0));

        String displayName;
        Color color;

        RobotMode(String displayName, Color color) {
            this.displayName = displayName;
            this.color = color;
        }

        public String getDisplayName() {
            return displayName;
        }

        public String getColor() {
            return color.toHexString();
        }
    };

    public enum WantedState {
        IDLE("kIdle"),
        CORAL_GROUND_PICKUP("kCoralGroundPickup"),
        ALGAE_GROUND_PICKUP("kAlgaeGroundPickup"),
        L1("kL1"),
        L2("kL2"),
        L3("kL3"),
        L4("kL4"),
        CLIMB("kClimb");

        String displayName;

        WantedState(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    public enum RobotState {
        IDLE("kIdle"),
        ALGAE_LOW_PICKUP("kAlgaeLowPickup"),
        ALGAE_HIGH_PICKUP("kAlgaeHighPickup"),
        ALGAE_GROUND_PICKUP("kAlgaeGroundPickup"),
        ALGAE_MARK_PICKUP("kAlgaeMarkPickup"),
        ALGAE_PROCESSOR_SCORE("kProcessorScore"),
        ALGAE_BARGE_SCORE("kBargeScore"),
        CORAL_GROUND_PICKUP("kCoralGroundPickup"),
        CORAL_L1_SCORE("kL1Scoring"),
        CORAL_L2_SCORE("kL2Scoring"),
        CORAL_L3_SCORE("kL3Scoring"),
        CORAL_L4_SCORE("kL4Scoring"),
        CLIMB("kClimb");

        String displayName;

        RobotState(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }

    }

    /*** Basic collection of tasks "modes" that the robot has.
     * Used in state logic to decide what state the robot needs to be in.
     */
    public RobotMode robotMode = RobotMode.IDLE;

    /***
     * State used in controls.
     */
    public WantedState wantedState = WantedState.IDLE;

    /*** A more detailed state of the robot, which instructs the different subsystems to perform actions. */
    public RobotState robotState = RobotState.IDLE;

    public Drivetrain drivetrain;
    public Effector effector;
    public Elevator elevator;
    public Intake intake;

    // NTPublishers
    StringPublisher robotModePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Superstructure/RobotMode").publish();
    StringPublisher wantedStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Superstructure/WantedState").publish();
    StringPublisher robotStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Superstructure/RobotState").publish();
    StringPublisher robotModeColorPublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Superstructure/RobotModeColor").publish();

    // Triggers
    public Trigger isEnabled = new Trigger(() -> DriverStation.isEnabled());
    public Trigger isDisabled = isEnabled.negate();
    public Trigger isTeleop = new Trigger(() -> DriverStation.isTeleop());
    public Trigger isTeleopEnabled = isEnabled.and(isTeleop);
    public Trigger isAuto = new Trigger(() -> DriverStation.isAutonomous());
    public Trigger isAutoEnabled = isEnabled.and(isAuto);
    public Trigger isEndgame = isTeleopEnabled.and(() -> DriverStation.getMatchTime() != -1 && DriverStation.getMatchTime() <= 20);

    public Trigger isIdleMode = new Trigger(() -> robotMode == RobotMode.IDLE);
    public Trigger isCoralScoringMode = new Trigger(() -> robotMode == RobotMode.CORAL);
    public Trigger isAlgaeScoringMode = new Trigger(() -> robotMode == RobotMode.ALGAE);
    public Trigger isClimbMode = new Trigger(() -> robotMode == RobotMode.CLIMB);
    
    
    /*** Tells each subsystem what it's task is currently/how to respond to it's own wanted states. */
    public Superstructure(Drivetrain drivetrain, Effector effector, Elevator elevator, Intake intake, Vision vision) {
        this.drivetrain = drivetrain;
        this.effector = effector;
        this.elevator = elevator;
        this.intake = intake;

        drivetrain.provideSubsystemAccessToSuperstructure(this);
        effector.provideSubsystemAccessToSuperstructure(this);
        elevator.provideSubsystemAccessToSuperstructure(this);
        intake.provideSubsystemAccessToSuperstructure(this);
        vision.provideSubsystemAccessToSuperstructure(this);

        setupTriggers();
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Effector getEffector() {
        return effector;
    }

    public Elevator getElevator() {
        return elevator;
    }

    public Intake getIntake() {
        return intake;
    }

    public void setupTriggers() {
        isDisabled.onTrue(setState(WantedState.IDLE));
        
        isEndgame.onTrue(
            Constants.OperatorConstants.driverController.blinkRumble(0.5, RumbleType.kBothRumble, 0.2)
            .alongWith(Constants.OperatorConstants.operatorController.blinkRumble(0.5, RumbleType.kBothRumble, 0.2)));
    }

    public void handleStateTransition() {
        if (getRobotMode() == RobotMode.ALGAE) {
            if (wantedState == WantedState.L1) {
                robotState = RobotState.ALGAE_PROCESSOR_SCORE;
            } else if (wantedState == WantedState.L4) {
                robotState = RobotState.ALGAE_BARGE_SCORE;
            } else if (wantedState == WantedState.IDLE) {
                robotState = RobotState.IDLE;
            }
        } else if (getRobotMode() == RobotMode.CORAL) {
            if (wantedState == WantedState.L1) {
                robotState = RobotState.CORAL_L1_SCORE;
            } else if (wantedState == WantedState.L2) {
                robotState = RobotState.CORAL_L2_SCORE;
            } else if (wantedState == WantedState.L3) {
                robotState = RobotState.CORAL_L3_SCORE;
            } else if (wantedState == WantedState.L4) {
                robotState = RobotState.CORAL_L4_SCORE;
            } else if (wantedState == WantedState.IDLE) {
                robotState = RobotState.IDLE;
            }
        } else if (getRobotMode() == RobotMode.IDLE) {
            if (wantedState == WantedState.ALGAE_GROUND_PICKUP || wantedState == WantedState.L1) {
                robotState = RobotState.ALGAE_GROUND_PICKUP;
            } else if (wantedState == WantedState.CORAL_GROUND_PICKUP) {
                robotState = RobotState.CORAL_GROUND_PICKUP;
            } else if (wantedState == WantedState.L2) {
                robotState = RobotState.ALGAE_LOW_PICKUP;
            } else if (wantedState == WantedState.L3) {
                robotState = RobotState.ALGAE_HIGH_PICKUP;
            } else if (wantedState == WantedState.IDLE) {
                robotState = RobotState.IDLE;
            }
        }
        applyState();
    }

    /*** Sets the overall scoring mode of the robot.
     * @param state The state to set.
     * @return Command to run that sets state.
     */
    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    /*** Sets the mode of the robot.
     * @param state The mode to set.
     */
    public void setRobotMode(RobotMode mode) {
        this.robotMode = mode;
    }

    public void toggleMode() {
        if (robotMode == RobotMode.ALGAE) {
            setRobotMode(RobotMode.CORAL);
        } else if (robotMode == RobotMode.CORAL) {
            setRobotMode(RobotMode.IDLE);
        } else if (robotMode == RobotMode.IDLE) {
            setRobotMode(RobotMode.ALGAE);
        }
    }

    /*** Sets the state of the robot.
     * @param state The wanted state to set.
     * @return Command to run that sets state.
     */
    public Command setState(WantedState state) {
        return Commands.runOnce(() -> setWantedState(state));
    }

    public Command setMode(RobotMode mode) {
        return Commands.runOnce(() -> setRobotMode(mode));
    }

    /*** Acts as a fake disable, running stop method in all other subsystems. */
    public void stop() {
        effector.stop();
        elevator.stop();
    }

    public RobotMode getRobotMode() {
        return robotMode;
    }

    public WantedState getWantedState() {
        return wantedState;
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public Pose2d getRobotPose() {
        return drivetrain.getState().Pose;
    }

    public APTarget getAlignmentTarget() {
        return drivetrain.target;
    }

    public void applyState() {
        if (robotState == RobotState.ALGAE_BARGE_SCORE) {
            elevator.setState(State.NET);
        } else if (robotState == RobotState.ALGAE_GROUND_PICKUP) {
            elevator.setState(State.GROUND_ALGAE_INTAKE);
        } else if (robotState == RobotState.ALGAE_HIGH_PICKUP) {
            elevator.setState(State.HIGH_ALGAE_INTAKE);
        } else if (robotState == RobotState.ALGAE_LOW_PICKUP) {
            elevator.setState(State.LOW_ALGAE_INTAKE);
        } else if (robotState == RobotState.ALGAE_PROCESSOR_SCORE) {
            elevator.setState(State.PROCESSOR);
        } else if (robotState == RobotState.CORAL_GROUND_PICKUP) {
            elevator.setState(State.CORAL_INTAKE);
        } else if (robotState == RobotState.CORAL_L1_SCORE) {
            elevator.setState(State.L1);
        } else if (robotState == RobotState.CORAL_L2_SCORE) {
            elevator.setState(State.L2);
        } else if (robotState == RobotState.CORAL_L3_SCORE) {
            elevator.setState(State.L3);
        } else if (robotState == RobotState.CORAL_L4_SCORE) {
            elevator.setState(State.L4);
        } else if (robotState == RobotState.IDLE) {
            elevator.setState(State.STOWED);
        }
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
        handleStateTransition();

        robotModePublisher.set(robotMode.getDisplayName());
        wantedStatePublisher.set(wantedState.getDisplayName());
        robotStatePublisher.set(robotState.getDisplayName());
        robotModeColorPublisher.set(robotMode.getColor());
    }

}
