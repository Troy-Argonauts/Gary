package org.troyargonauts.robot;

import edu.wpi.first.math.util.Units;
import org.troyargonauts.common.motors.wrappers.MotorController;
import org.troyargonauts.common.util.Gains;

public final class Constants {

    public interface Controllers {
        int DRIVER = 0;
        int OPERATOR = 1;
    }

    public interface Limelight {
        double LIMELIGHT_HEIGHT = 34;
        double MOUNTING_ANGLE = 13;
        double[] APRIL_TAG_HEIGHTS_BY_ID = {61, 48.125, 48.125, 51.875, 51.875, 48.125, 48.125, 51.875, 51.875, 48.125, 48.125, 47.5, 47.5, 47.5, 47.5};

        double STOP_THRESHOLD = 0.9;
    }
}
