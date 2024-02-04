package org.troyargonauts.robot;

import edu.wpi.first.math.util.Units;
import org.troyargonauts.common.motors.wrappers.MotorController;
import org.troyargonauts.common.util.Gains;

public final class Constants {

    public interface Controllers {
        int DRIVER = 0;
        int OPERATOR = 1;
    }

    public interface Climber
    {
        int MOTOR_ID = 13;
        String CANBUS_NAME = "roborio";
        double CLIMBER_MOTOR_OFF_SPEED = 0.0;
        double CLIMBER_MOTOR_ON_SPEED = 0.75;
        double DISTANCE_MIN = 1.0;
        double DISTANCE_MAX = 2.0;
        int CLIMBER_MOTOR_PORT = 13;
        String CLIMBER_CANBUS_NAME = "roborio";
    }
}
