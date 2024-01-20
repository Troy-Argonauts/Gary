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
        double LIMELIGHT_HEIGHT = 32.625;
        double APRIL_TAG_HEIGHT = 18;
        double MOUNTING_ANGLE = 0;

        double LOW_CONE_HEIGHT = 23;

        double HIGH_CONE_HEIGHT = 43;

        double DISTANCE_BETWEEN_CONES = 17;

    }
}
