package org.troyargonauts.robot;

import edu.wpi.first.math.util.Units;
import org.troyargonauts.common.motors.wrappers.MotorController;
import org.troyargonauts.common.util.Gains;

public final class Constants {

    public interface Controllers {
        int DRIVER = 0;
        int OPERATOR = 1;
    }

    public interface Shooter {

        public int TOP_MOTOR_ID = 1;
        public int BOTTOM_MOTOR_ID = 2;

        double TOP_MOTOR_P = 0;
        double TOP_MOTOR_I = 0;
        double TOP_MOTOR_D = 0;
        double BOTTOM_MOTOR_P = 0;
        double BOTTOM_MOTOR_I = 0;
        double BOTTOM_MOTOR_D = 0;

        String CANBUS_NAME = "roborio";
    }
}
