package org.troyargonauts.robot;

import edu.wpi.first.math.util.Units;
import org.troyargonauts.common.motors.wrappers.MotorController;
import org.troyargonauts.common.util.Gains;

public final class Constants {

    public interface Controllers {
        int DRIVER = 0;
        int OPERATOR = 1;
    }

    public interface Arm {
        double P = 0;
        double I = 0;
        double D = 0;
        int LEFT_MOTOR_ID = 0;
        int RIGHT_MOTOR_ID = 1;
        double DEADBAND = 0.08;
        String CANBUS_NAME = "roborio";
    }
}
