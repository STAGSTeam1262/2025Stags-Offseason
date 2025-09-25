package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MotorIDConstants;

public class Climber extends SubsystemBase {

    Superstructure superstructure;

    public enum ClimberPosition {
        STOWED,
        OUT,
        CLIMBING,
        PULL_BACK;
    }

    // TalonFX climberMotor = new TalonFX(MotorIDConstants.climberID, "Canivore");

    public ClimberPosition position = ClimberPosition.STOWED;
    public StringPublisher positionPublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Climber/Climber Position").publish();

    Trigger enableTrigger = new Trigger(() -> DriverStation.isEnabled()).onTrue(setPosition(ClimberPosition.OUT));
    
    public Climber() {

    }

    public void setClimberPosition(ClimberPosition position) {
        this.position = position;
    }

    public Command setPosition(ClimberPosition position) {
        return Commands.runOnce(() -> setClimberPosition(position));
    }

    public void handleClimberMovement() {
        if (position == ClimberPosition.STOWED) {

        } else if (position == ClimberPosition.OUT) {
            
        } else if (position == ClimberPosition.CLIMBING) {

        } else if (position == ClimberPosition.PULL_BACK) {
            
        } else {

        }
    }

    public boolean nextClimberPosition() {
        if (position == ClimberPosition.OUT) {
            setClimberPosition(ClimberPosition.CLIMBING);
            return true;
        } else if (position == ClimberPosition.CLIMBING) {
            setClimberPosition(ClimberPosition.PULL_BACK);
            return true;
        } else if (position == ClimberPosition.PULL_BACK) {
            setClimberPosition(ClimberPosition.OUT);
            return false;
        } else {
            return false;
        }
    }

    public void stop() {
        // climberMotor.stopMotor();
    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        positionPublisher.set(position.toString());
    }

}
