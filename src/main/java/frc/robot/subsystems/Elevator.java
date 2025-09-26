package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

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

    TalonFX motorA = new TalonFX(Constants.MotorIDConstants.elevatorAID, "Canivore");
    TalonFX motorB = new TalonFX(Constants.MotorIDConstants.elevatorBID, "Canivore");

    DoublePublisher motorPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Elevator/Position").publish();
    DoublePublisher motorBPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Elevator/Position B").publish();

    VelocityVoltage voltageRequest = new VelocityVoltage(0).withFeedForward(0).withSlot(1);
    MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    public State state = State.IDLE;
    StringPublisher statePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Elevator/State").publish();

    public double setpoint = 0;
    DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Elevator/Setpoint").publish();

    public boolean manual = false;

    public Trigger isAtSetpoint = new Trigger(() -> motorA.getPosition().getValueAsDouble() <= setpoint + 2 && motorA.getPosition().getValueAsDouble() >= setpoint - 2).or(() -> manual);

    
    /*** The end effector of the robot. */
    public Elevator() {
        configureMotors();

        motorA.setPosition(0);
        motorB.setPosition(0);
    }

    public void configureMotors() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicCruiseVelocity(60).withMotionMagicAcceleration(200);
        Slot0Configs slot0Configs = new Slot0Configs()
            .withKP(4.8)
            .withKI(0.0)
            .withKD(0.1)
            .withKS(0.25)
            .withKV(0.12)
            .withKA(0.01)
            .withKG(0.0);
        Slot1Configs slot1Configs = new Slot1Configs()
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKG(-2.3);
        TalonFXConfiguration config = new TalonFXConfiguration().withMotionMagic(mmConfigs).withSlot0(slot0Configs).withSlot1(slot1Configs).withMotorOutput(motorOutputConfigs);
        motorA.getConfigurator().apply(config);
        config = config.withMotorOutput(motorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive));
        motorB.getConfigurator().apply(config);
    }

    public void stop() {
        motorA.stopMotor();
        motorB.stopMotor();
    }

    public void setState(State state) {
        this.state = state;
        handleStateTransition();
    }

    public void setVoltage(double voltage) {
        motorA.setControl(voltageRequest.withFeedForward(voltage));
        motorB.setControl(voltageRequest.withFeedForward(voltage));
    }

    public Command set(double voltage) {
        return Commands.runOnce(() -> setVoltage(voltage));
    }

    public void moveToSetpoint(double setpoint) {
        motorA.setControl(motionMagicRequest.withPosition(setpoint));
        motorB.setControl(motionMagicRequest.withPosition(setpoint));
        this.setpoint = setpoint;
        manual = false;
    }

    public Command setElevatorState(State state) {
        return Commands.runOnce(() -> setState(state));
    }

    public void handleStateTransition() {
        if (state == State.STOWED) {
            moveToSetpoint(0);
        } else if (state == State.L2) {
            moveToSetpoint(-20);
        } else if (state == State.L3) {
            moveToSetpoint(-31);
        } else if (state == State.L4) {
            moveToSetpoint(-48);
        } else if (state == State.NET) {
            moveToSetpoint(-54);
        }
    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        statePublisher.set(state.toString());

        setpointPublisher.set(setpoint);
        motorPositionPublisher.set(motorA.getPosition().getValueAsDouble());
        motorBPositionPublisher.set(motorB.getPosition().getValueAsDouble());
    }

}
