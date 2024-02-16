package org.troyargonauts.robot;

public final class Constants {
    public interface Intake {
        int MOTOR_CAN_ID = 5;
        int NOTE_SENSOR_SLOT = 0;

        String CANBUS_NAME = "roborio";
    }

    public interface Controllers {
        int DRIVER = 0;
        int OPERATOR = 1;

        double DEADBAND = 0.08;
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
  
    public interface Climber {
        int MOTOR_ID = 13;
        String CANBUS_NAME = "roborio";
        double P = 0;
        double I = 0;
        double D = 0;

        double DEADBAND = 0.08;
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
