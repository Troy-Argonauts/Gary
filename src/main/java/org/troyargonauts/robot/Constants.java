package org.troyargonauts.robot;

public final class Constants {

    public interface Controllers {
        int DRIVER = 0;
        int OPERATOR = 1;
    }

    public interface Climber
    {
        int MOTOR_ID = 13;
        String CANBUS_NAME = "roborio";
        double P = 0;
        double I = 0;
        double D = 0;
    }
}
