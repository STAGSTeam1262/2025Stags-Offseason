package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    public enum WantedState {
        IDLE("kIdle"),
        STOWED("kStow"),
        CORAL_INTAKE("kIntake"),
        L1("kL1"),
        L2("kL2"),
        L3("kL3"),
        L4("kL4");

        String displayName;

        WantedState(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    public enum State {
        IDLE("kIdle"),
        STOWED("kStow"),
        CORAL_INTAKE("kCoralIntake"),
        GROUND_ALGAE_INTAKE("kGroundAlgaeIntake"),
        LOW_ALGAE_INTAKE("kLowAlgaeIntake"),
        HIGH_ALGAE_INTAKE("kHighAlgaeIntake"),
        L1("kL1"),
        L2("kL2"),
        L3("kL3"),
        L4("kL4"),
        PROCESSOR("kProcessor"),
        NET("kNet");

        String displayName;

        State(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    Superstructure superstructure;

    public WantedState wantedState = WantedState.IDLE;
    StringPublisher wantedStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Elevator/WantedState").publish();
    public State state = State.IDLE;
    StringPublisher statePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Elevator/State").publish();

    public double setpoint = 0;
    DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Elevator/Setpoint").publish();

    
    /*** The end effector of the robot. */
    public Elevator() {

    }

    public void stop() {
        
    }

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    public Command setState(WantedState state) {
        return Commands.runOnce(() -> setWantedState(state));
    }

    public void handleStateTransition() {

    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        wantedStatePublisher.set(wantedState.getDisplayName());
        statePublisher.set(state.getDisplayName());

        setpointPublisher.set(setpoint);
    }

}
