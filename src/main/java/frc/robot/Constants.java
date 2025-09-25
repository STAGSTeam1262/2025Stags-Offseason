package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

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
        public static final APConstraints apConstraints = new APConstraints(2, 10, 5);
        public static final APProfile apProfile = new APProfile(apConstraints)
            .withErrorXY(Centimeters.of(2))
            .withErrorTheta(Degrees.of(0.5))
            .withBeelineRadius(Centimeters.of(8));
        public static final Autopilot autopilot = new Autopilot(apProfile);
    }

    public static class MotorIDConstants {
        public static final int elevatorAID = 9;
        public static final int elevatorBID = 10;

        public static final int effectorPivotID = 11;
        public static final int effectorWheelID = 12;
        public static final int funnelConveyorID = 13;
        
        public static final int climberID = 14;
    }
}
