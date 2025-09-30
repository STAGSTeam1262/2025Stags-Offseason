package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.RobotMode;
import frc.robot.subsystems.Superstructure.RobotState;

public class Effector extends SubsystemBase {

    public enum PivotState {
        STOWED,
        ALGAE,
        PROCESSOR_SCORE;
    }

    public enum WheelState {
        IDLE,
        CORAL_INTAKE,
        ALGAE_INTAKE,
        EJECT;
    }

    TalonFX wheelMotor = new TalonFX(Constants.MotorIDConstants.effectorWheelID, "Canivore");
    TalonFX pivotMotor = new TalonFX(Constants.MotorIDConstants.effectorPivotID, "Canivore");
    TalonFX funnelMotor = new TalonFX(Constants.MotorIDConstants.funnelConveyorID, "Canivore");

    Superstructure superstructure;

    public PivotState pivotState = PivotState.STOWED;
    StringPublisher pivotStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Effector/PivotState").publish();
    public WheelState wheelState = WheelState.IDLE;
    StringPublisher wheelStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Effector/WheelState").publish();

    DoublePublisher pivotPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Effector/Pivot Position").publish();

    DoublePublisher wheelVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Effector/Wheel Voltage").publish();
    public double pivotSetpoint = 0;
    DoublePublisher pivotSetpointPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Effector/Pivot Setpoint").publish();

    MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    // Trigger setup = new Trigger(() -> DriverStation.isEnabled()).onTrue(setPivotState(PivotState.STOWED));
    
    /*** The end effector of the robot. */
    public Effector() {
        configureMotors();
        pivotMotor.setPosition(0);
    }

    public void configureMotors() {
        MotionMagicConfigs mmConfig = new MotionMagicConfigs().withMotionMagicCruiseVelocity(80).withMotionMagicAcceleration(160);
        Slot0Configs slot0Configs = new Slot0Configs()
            .withKP(4.8)
            .withKI(0.0)
            .withKD(0.1)
            .withKS(0.25)
            .withKV(0.12)
            .withKA(0.01)
            .withKG(0.0);
        MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration config = new TalonFXConfiguration().withMotionMagic(mmConfig).withSlot0(slot0Configs).withMotorOutput(outputConfig);
        pivotMotor.getConfigurator().apply(config);
    }

    public void stop() {
        pivotMotor.stopMotor();
        wheelMotor.stopMotor();
    }

    public void setState(PivotState state) {
        pivotState = state;
        handlePivotState();
    }

    public Command setPivotState(PivotState state) {
        return Commands.runOnce(() -> setState(state));
    }

    public void setRollerState(WheelState state) {
        wheelState = state;
        handleWheelState();
    }

    public Command setWheelState(WheelState state) {
        return Commands.runOnce(() -> setRollerState(state));
    }

    public void setPivotVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    public Command setPivotVolts(double voltage) {
        return Commands.runOnce(() -> setPivotVoltage(voltage));
    }

    public PivotState getPivotState() {
        return pivotState;
    }

    public WheelState getWheelState() {
        return wheelState;
    }

    public void handlePivotState() {
        if (pivotState == PivotState.ALGAE) {
            movePivotToSetpoint(8);
        } else if (pivotState == PivotState.STOWED) {
            movePivotToSetpoint(0);
        } else if (pivotState == PivotState.PROCESSOR_SCORE) {
            movePivotToSetpoint(12.5);
        }
    }

    public Command autoEjectCoral() {
        return Commands.runOnce(() -> setWheelVoltage(-6));
    }

    public void handleWheelState() {
        if (wheelState == WheelState.ALGAE_INTAKE) {
            if (superstructure.robotState == RobotState.ALGAE_HIGH_PICKUP || superstructure.robotState == RobotState.ALGAE_LOW_PICKUP) {
                setState(PivotState.ALGAE);
            }
            setConveyorVoltage(-5);
            setWheelVoltage(-6);
        } else if (wheelState == WheelState.CORAL_INTAKE) {
            setWheelVoltage(-5);
            setConveyorVoltage(7);
        } else if (wheelState == WheelState.EJECT) {
            if (superstructure.robotMode == RobotMode.CORAL) {
                setWheelVoltage(-6);
                setConveyorVoltage(0);
            } else if (superstructure.robotMode == RobotMode.IDLE) {
                if (superstructure.robotState == RobotState.ALGAE_PROCESSOR_SCORE) {
                    setWheelVoltage(6);
                    setConveyorVoltage(0);
                    setState(PivotState.PROCESSOR_SCORE);
                } else if (superstructure.robotState == RobotState.ALGAE_BARGE_SCORE) {
                    setWheelVoltage(6);
                    setConveyorVoltage(0);
                }
            }
        } else if (wheelState == WheelState.IDLE) {
            setWheelVoltage(0);
            setConveyorVoltage(0);
        } else {
            setWheelVoltage(0);
            setConveyorVoltage(0);
        }
    }

    public void togglePivot() {
        if (pivotState == PivotState.ALGAE) {
            setState(PivotState.STOWED);
        } else if (pivotState == PivotState.STOWED) {
            setState(PivotState.ALGAE);
        } else {
            setState(PivotState.ALGAE);
        }
    }

    public Command pivotToggle() {
        return Commands.runOnce(() -> togglePivot());
    }

    public void movePivotToSetpoint(double setpoint) {
        pivotMotor.setControl(motionMagicRequest.withPosition(setpoint));
    }

    public void setWheelVoltage(double voltage) {
        wheelMotor.setVoltage(voltage);
    }

    public void setConveyorVoltage(double voltage) {
        funnelMotor.setVoltage(voltage);
    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        pivotStatePublisher.set(pivotState.toString());
        wheelStatePublisher.set(wheelState.toString());

        // wheelVoltagePublisher.set(wheelMotor.getMotorVoltage().getValueAsDouble());
        pivotSetpointPublisher.set(pivotSetpoint);
        pivotPositionPublisher.set(pivotMotor.getPosition().getValueAsDouble());
    }

}
