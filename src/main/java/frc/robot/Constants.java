package frc.robot;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;

import frc.robot.utils.Controller;

public final class Constants {
    
    /*** Contains controllers. */
    public static class OperatorConstants {
        public static final Controller driverController = new Controller(0);
        public static final Controller operatorController = new Controller(1);
    }

    public static class AutopilotConstants {
        public static final APConstraints apConstraints = new APConstraints(6, 10, 5);
        public static final APProfile apProfile = new APProfile().withConstraints(apConstraints);
        public static final Autopilot autopilot = new Autopilot(apProfile);
    }
}
