package org.troyargonauts.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.robot.Constants;





public class Limelight extends SubsystemBase {

    /**
     * The current pipeline number being used by the Limelight.
     */
    public static int pipeline = 0;
    private NetworkTableInstance table = null;

    /**
     * The different modes the Limelight's LED can be set to.
     */
    public enum LightMode {
        ON, OFF, BLINK
    }
    /**
     * The different modes the Limelight's camera can be set to.
     */
    public enum CameraMode {
        VISION, DRIVER
    }



    private final static Limelight INSTANCE = new Limelight();


    /**
     * Creates an instance of the Limelight subsystem and sets the initial LED mode to ON and the camera mode to VISION.
     */
    public Limelight() {
//        limelight.setLedMode(Limelight.LightMode.ON);
//        limelight.setCameraMode(Limelight.CameraMode.VISION);
    }


    /**
     * Returns the instance of the Limelight subsystem.
     *
     * @return The instance of the Limelight subsystem.
     */
    public static Limelight getInstance() {
        return INSTANCE;
    }




    /**
     * Returns the Ty (vertical offset from the crosshair to the target) value from the Limelight camera.
     *
     * @return The Ty(vertical offset from the crosshair to the target) value from the Limelight camera.
     */

    public double getTy() {
        return getValue("ty").getDouble(0.00);
    }

    /**
     * Returns the Tx (horizontal offset from the crosshair to the target) value from the Limelight camera.
     * @return Tx (horizontal offset from the crosshair to the target) value from the Limelight camera.
     */
    public double getTx(){
        return getValue("tx").getDouble(0.00);
    }

    public double getXAngle(){
        return (getValue("tx").getDouble(0.00)) * -2;
    }

    /**
     * Calculates and returns the distance from the Limelight to the AprilTag target in inches.
     *
     * @return The distance from the Limelight to the AprilTag target in inches.
     */

    public double getDistanceFromAprilTagInches()  {
        double angle = Math.toRadians(Constants.Limelight.MOUNTING_ANGLE) + getTy();
        if (angle < 1 || angle > 89) {
            return 0;
        }
        double tan = Math.tan(Math.toRadians(angle));
        return ((Limelight.getAprilTagHeight(0) - Constants.Limelight.LIMELIGHT_HEIGHT) / tan)/2.54;

    }



    /**
     * Returns the network table entry for a given key in the Limelight table.
     *
     * @param key The key of the network table entry to retrieve.
     * @return The network table entry for the given key in the Limelight table.
     */

    private NetworkTableEntry getValue(String key) {
        if (table == null) {
            table = NetworkTableInstance.getDefault();
        }

        return table.getTable("limelight").getEntry(key);
    }


    /**
     * Sets the LED mode of the Limelight.
     *
     * @param mode the new LED mode to set
     */
    public void setLedMode(LightMode mode) {
        getValue("ledMode").setNumber(mode.ordinal());
    }

    /**
     * Sets the camera mode of the Limelight.
     *
     * @param mode the new camera mode to set
     */
    public void setCameraMode(CameraMode mode) {
        getValue("camMode").setNumber(mode.ordinal());
    }

    /**
     * Sets the pipeline number of the Limelight.
     *
     */
    private void setPipeline(int pipelineID) {
        getValue("pipeline").setNumber(pipeline);
        pipeline = pipelineID;
    }

    /**
     * Returns ID of current pipeline
     * @return current pipeline number
     */
    public int getPipeline(){
        if(pipeline != getValue("getPipe").getInteger(0)){
            pipeline = (int) getValue("getPipe").getInteger(0);
        }
        return pipeline;
    }

    /**
     * returns height of April Tag on field with given ID
     * @param id April Tag ID
     * @return height of April Tag with given ID in inches
     */
    private static double getAprilTagHeight(int id){
        return Constants.Limelight.APRIL_TAG_HEIGHTS_BY_ID[id];
    }

    /**
     * If robot is detected via object detection pipeline, returns area coverage of screen.
     * @return area of screen covered by a detected robot.
     */
    public double getAreaOfDetectedRobot(){
        if (pipeline == 1){
            if(getValue("tclass").toString().equals("robot") && (((int) getValue("tv").getInteger(0)) == 1)){
                return getValue("ta").getDouble(0);
            }
            return 0;
        }
        else{
            return 0;
        }
    }

    /**
     * This method displays the April Tag Distance,  getDistanceFromAprilTagInches,
     * methods respectively. Get pipeline displays current Pipeline;
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("April Tag Distance", getDistanceFromAprilTagInches());
        SmartDashboard.putNumber("Angle from AprilTag: ", getXAngle());
        SmartDashboard.putNumber("Current Pipeline", getPipeline());
    }


}