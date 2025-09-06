package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {

    public enum State {
        IDLE,
        STOWED,
        CORAL_INTAKE,
        GROUND_ALGAE_INTAKE,
        LOW_ALGAE_INTAKE,
        HIGH_ALGAE_INTAKE,
        L1,
        L2,
        L3,
        L4,
        PROCESSOR,
        NET;
    }

    Superstructure superstructure;
    public State state = State.IDLE;
    StringPublisher statePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Elevator/State").publish();

    public double setpoint = 0;
    DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Elevator/Setpoint").publish();

    public double motorPosition = 0;
    public Trigger isAtSetpoint = new Trigger(() -> motorPosition <= setpoint + 2 && motorPosition >= setpoint - 2);

    
    /*** The end effector of the robot. */
    public Elevator() {
        
    }

    public void stop() {
        
    }

    public void setState(State state) {
        this.state = state;
    }

    public Command setElevatorState(State state) {
        return Commands.runOnce(() -> setState(state));
    }

    public void handleStateTransition() {

    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        statePublisher.set(state.toString());

        setpointPublisher.set(setpoint);
    }

}
