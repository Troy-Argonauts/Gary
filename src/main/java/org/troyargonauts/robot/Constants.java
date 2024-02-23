package org.troyargonauts.robot;

public final class Constants {
    public interface Intake {
        int MOTOR_CAN_ID = 22;

        int MOTOR_CAN_ID2 = 26;

        int NOTE_SENSOR_SLOT = 0;

        String CANBUS_NAME = "roborio";

        String CANBUS_NAME2 = "roborio2";
    }

    public interface Controllers {
        int DRIVER = 0;
        int OPERATOR = 1;

        double DEADBAND = 0.08;
    }

    public interface Arm {
        int LEFT_MOTOR_ID = 20;
        int RIGHT_MOTOR_ID = 21;

        int LIMIT_SWITCH_SLOT = 1;

        double P = 0;
        double I = 0;
        double D = 0;

        String CANBUS_NAME = "roborio";
    }
  
    public interface Climber {
        int MOTOR_ID = 23;

        double P = 0;
        double I = 0;
        double D = 0;

        String CANBUS_NAME = "roborio";
    }
  
    public interface Shooter {
        int TOP_MOTOR_ID = 24;
        int BOTTOM_MOTOR_ID = 25;

        double TOP_MOTOR_P = 0;
        double TOP_MOTOR_I = 0;
        double TOP_MOTOR_D = 0;
        double BOTTOM_MOTOR_P = 0;
        double BOTTOM_MOTOR_I = 0;
        double BOTTOM_MOTOR_D = 0;

        String CANBUS_NAME = "roborio";
    }  
}
