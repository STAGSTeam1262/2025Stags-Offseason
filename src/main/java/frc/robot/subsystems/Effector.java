package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Effector extends SubsystemBase {

    public enum PivotState {
        IDLE("kIdle");

        String displayName;

        PivotState(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    public enum WheelState {
        IDLE("kIdle"),
        ALGAE_INTAKE("kAlgaeIntake"),
        EJECT("kEject");

        String displayName;

        WheelState(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    Superstructure superstructure;

    public PivotState pivotState = PivotState.IDLE;
    StringPublisher pivotStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Effector/PivotState").publish();
    public WheelState wheelState = WheelState.IDLE;
    StringPublisher wheelStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Effector/WheelState").publish();

    public double wheelVoltage = 0;
    DoublePublisher wheelVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Effector/Wheel Voltage").publish();
    public double pivotSetpoint = 0;
    DoublePublisher pivotSetpointPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Effector/Pivot Setpoint").publish();
    
    /*** The end effector of the robot. */
    public Effector() {

    }

    public void stop() {
        
    }

    public void setState(PivotState state) {
        this.pivotState = state;
    }

    public Command setPivotState(PivotState state) {
        return Commands.runOnce(() -> setState(state));
    }

    public void setRollerState(WheelState state) {
        this.wheelState = state;
    }

    public Command setWheelState(WheelState state) {
        return Commands.runOnce(() -> setRollerState(state));
    }

    public PivotState getPivotState() {
        return pivotState;
    }

    public WheelState getWheelState() {
        return wheelState;
    }

    public void handlePivotStateTransition() {

    }

    public void handleWheelState() {

    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        pivotStatePublisher.set(pivotState.getDisplayName());
        wheelStatePublisher.set(wheelState.getDisplayName());

        wheelVoltagePublisher.set(wheelVoltage);
        pivotSetpointPublisher.set(pivotSetpoint);
    }

}
