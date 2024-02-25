package org.troyargonauts.robot;

/**
 * Class containing all constants for all subsystems except for CommandSwerveDrivetrain
 */
public class Constants {
    /**
     * Constants for the Intake subsystem
     */
    public interface Intake {
        int LEFT_MOTOR_CAN_ID = 22;

        int RIGHT_MOTOR_CAN_ID = 26;

        int NOTE_SENSOR_SLOT = 0;

        String CANBUS_NAME = "roborio";
    }

    /**
     * Constants for the Controllers
     */
    public interface Controllers {
        int DRIVER = 0;
        int OPERATOR = 1;

        double DEADBAND = 0.08;
    }

    /**
     * Constants for the Arm subsystem
     */
    public interface Arm {
        int LEFT_MOTOR_ID = 20;
        int RIGHT_MOTOR_ID = 21;

        int LIMIT_SWITCH_SLOT = 1;

        double P = 0.75;
        double I = 0;
        double D = 0;

        String CANBUS_NAME = "roborio";
    }

    /**
     * Constants for the Climber subsystem
     */
    public interface Climber {
        int MOTOR_ID = 23;

        double P = 0;
        double I = 0;
        double D = 0;

        String CANBUS_NAME = "roborio";
    }

    /**
     * Constants for the Shooter subsystem
     */
    public interface Shooter {
        int TOP_MOTOR_ID = 24;
        int BOTTOM_MOTOR_ID = 25;

        double TOP_MOTOR_P = 0.457;
        double TOP_MOTOR_I = 0.21;
        double TOP_MOTOR_D = 0.0028;
        double BOTTOM_MOTOR_P = 0.457;
        double BOTTOM_MOTOR_I = 0.21;
        double BOTTOM_MOTOR_D = 0.0028;

        String CANBUS_NAME = "roborio";
    }

    public interface Drivetrain{

        double MAX_SPEED = 4.572; // 6 meters per second desired top speed
        double MAX_ANGULAR_RATE = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    }
    
}
