package org.troyargonauts.robot;

import edu.wpi.first.math.util.Units;
import org.troyargonauts.common.motors.wrappers.MotorController;
import org.troyargonauts.common.util.Gains;

public final class Constants {
    public interface Intake{
        int MOTOR_CAN_ID = 1;
        int NOTE_SENSOR_SLOT = 2;
        String INTAKE_CANBUS_NAME = "roborio";
    }

    public interface Controllers {
        int DRIVER = 0;
        int OPERATOR = 1;
    }
}
