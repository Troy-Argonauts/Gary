package org.troyargonauts.robot;

/**
 * Class containing all constants for all subsystems except for CommandSwerveDrivetrain
 */
public final class Constants {
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

        double UP_P = 0.6; //0.83
        double UP_I = 0;
        double UP_D = 0;
        double DOWN_P = 0.1; //0.25
        double DOWN_I = 0;
        double DOWN_D = 0;
        double ZERO = 0.0;

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

        double P = 0.457;
        double I = 0.217;
        double D = 0.0018;


        String CANBUS_NAME = "roborio";
    }

    public interface Drivetrain{

        double MAX_SPEED = 4.572; // 6 meters per second desired top speed
        double MAX_ANGULAR_RATE = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    }
}
