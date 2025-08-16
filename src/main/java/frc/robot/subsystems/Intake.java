package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public enum IntakeState {
        IDLE("kIdle"),
        LOWERED("kLowered"),
        STOWED("kStowed"),
        INTAKING("kIntaking"),
        INDEXING_STOWED("kIndexingStowed"),
        INDEXING_LOWERED("kIndexingLowered"),
        INTAKING_NO_INDEX("kIntakingNoIndex");
        
        String displayName;

        IntakeState(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    Superstructure superstructure;

    public IntakeState intakeState = IntakeState.IDLE;

    StringPublisher statePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Intake/State").publish();

    public double setpoint = 0;
    DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Intake/Setpoint").publish();

    public boolean isStowed = false;
    BooleanPublisher stowedPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Subsystems/Intake/Is Stowed").publish();
    
    public Intake() {

    }

    public void stop() {

    }

    public void handleStateTransition() {

    }

    public void setState(IntakeState state) {
        this.intakeState = state;
    }

    public Command setIntakeState(IntakeState state) {
        return Commands.runOnce(() -> setState(state));
    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        handleStateTransition();

        statePublisher.set(intakeState.getDisplayName());
    }

}
