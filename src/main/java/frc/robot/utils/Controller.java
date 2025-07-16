package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controller {

    CommandXboxController controller;

    /***
     * Class that adds additional controls for things like vibration and commands for sequencing rumble more easily.
     * 
     * @param id id of the controller being handled, which will create a new controller object, accessed through getController().
     */
    public Controller(int id) {
        controller = new CommandXboxController(id);
    }

    /***
     * Class that adds additional controls for things like vibration and commands for sequencing rumble more easily.
     * 
     * @param controller Directly passes in the controller being handled, rather than the id to create a new controller. The controller can be accessed by calling getController().
     */
    public Controller(CommandXboxController controller) {
        this.controller = controller;
    }

    public CommandXboxController getController() {
        return controller;
    }

    public boolean isConnected() {
        return controller.isConnected();
    }

    public void rumble(RumbleType type, double power) {
        controller.setRumble(type, power);
    }

    public void cancelRumble() {
        controller.setRumble(RumbleType.kBothRumble, 0);
    }

    public Command setRumble(RumbleType type, double power) {
        return Commands.runOnce(() -> rumble(type, power));
    }

    public Command stopRumble() {
        return Commands.runOnce(() -> cancelRumble());
    }

    public Command rumbleThenStop(RumbleType type, double power, double time) {
        return setRumble(type, power).andThen(new WaitCommand(time)).andThen(stopRumble());
    }

    public Command blinkRumble(double time, RumbleType type, double power) { 
        return rumbleThenStop(type, power, time)
            .andThen(new WaitCommand(time)).andThen(rumbleThenStop(type, power, time))
            .andThen(new WaitCommand(time)).andThen(rumbleThenStop(type, power, time));
    }
    
}
