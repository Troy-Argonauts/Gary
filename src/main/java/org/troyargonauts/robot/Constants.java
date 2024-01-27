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

        public double MOTOR_1_ENCODER = 0;
        public double MOTOR_2_ENCODER = 0;

        double MOTOR_1_P = 0;
        double MOTOR_1_I = 0;
        double MOTOR_1_D = 0;
        double MOTOR_2_P = 0;
        double MOTOR_2_I = 0;
        double MOTOR_2_D = 0;
        double MOTOR_3_P = 0;
        double MOTOR_3_I = 0;
        double MOTOR_3_D = 0;

        String SHOOTER_CANBUS_NAME = "roborio";
    }
}
