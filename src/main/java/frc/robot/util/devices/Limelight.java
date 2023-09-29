package frc.robot.util.devices;

import frc.robot.Constants.Misc;
import frc.robot.Constants.Vision;
import frc.robot.Constants.Vision.TARGET_TYPE;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private static Limelight instance;

    private NetworkTable networkTable;
    private TARGET_TYPE currentTarget;
    private double targetHeight;

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    private Limelight() {
        this.networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void setDesiredTarget(TARGET_TYPE target) {
        this.currentTarget = target;
        switch (target) {
            case REFLECTIVE_TAPE:
                this.targetHeight = Vision.REFLECTIVE_TAPE_HEIGHT;
                this.setPipeline(Vision.REFLECTIVE_PIPLINE);
                break;
            case APRILTAG:
                this.targetHeight = Vision.APRILTAG_HEIGHT;
                this.setPipeline(Vision.APRILTAG_PIPELINE);
                break;
            case CONE:
                this.targetHeight = Vision.CONE_HEIGHT;
                this.setPipeline(Vision.CONE_PIPELINE);
                break;
            case CUBE:
                this.targetHeight = Vision.CUBE_HEIGHT;
                this.setPipeline(Vision.CUBE_PIPELINE);
                break;
            default:
                this.targetHeight = 0;
                break;
        }
    }

    public TARGET_TYPE getDesiredTarget() {
        return this.currentTarget;
    }

    private void setPipeline(int pipeline) {
        this.networkTable.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Change the mode of the led [0-default, 1-off, 2-blink, 3-on]
     * 
     * @param ledMode led mode
     */
    public void setLedMode(int ledMode) {
        this.networkTable.getEntry("ledMode").setNumber(ledMode);
    }

    public double getTargetX() {
        return this.networkTable.getEntry("tx").getDouble(0);
    }

    public double getTargetY() {
        return this.networkTable.getEntry("ty").getDouble(0);
    }

    public boolean getTargetExists() {
        return this.networkTable.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetArea() {
        return this.networkTable.getEntry("ta").getDouble(0);
    }

    /**
     * Get the distance to the target using Trigonometry
     * 
     * @return distance to target
     */
    public double getTargetDistance() {
        return Math.abs(
                Math.round(
                        (Math.abs(Vision.LIMELIGHT_HEIGHT - this.targetHeight) /
                                Math.tan(Vision.LIMELIGHT_ANGLE - this.getTargetY())) * 10)
                        / 10); // distance from target in inches
    }

    /**
     * Checks if limelight reached target x angle
     * 
     * @return reached target
     */
    public boolean reachedTargetX() {
        return (Misc.WITHIN_TOLERANCE(this.getTargetX(), 0, Vision.LIMELIGHT_TOLERANCE));
    }
}