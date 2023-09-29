package frc.robot.util.control;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmPresets {

    // PID coefficients
    public double wristHeight;
    public double wristOffset;
    public String name;

    /**
     * PID Values for Basic Balancing Algorithm
     * 
     * @param wristHeight
     * @param wristOffset
     */
    public ArmPresets(double height, double offset, String name) {
        this.wristHeight = height;
        this.wristOffset = offset;
        this.name = name;
    }

    /**
     * Push preset properties to the Smart Dashboard
     * 
     * @param name
     */
    public void pushToDashboard() {
        SmartDashboard.putNumber(name + " Wrist Height", wristHeight);
        SmartDashboard.putNumber(name + " Wrist Offset", wristOffset);
    }

    /**
     * Retrieve preset properties from the Smart Dashboard
     * 
     * @param name
     */
    public void getFromDashboard() {
        this.wristHeight = SmartDashboard.getNumber(this.name + " Wrist Height", 0);
        this.wristOffset = SmartDashboard.getNumber(this.name + " Wrist Offset", 0);
    }

    /**
     * @return String representation of preset properties
     */
    @Override
    public String toString() {
        return String.format("Wrist Height: %f, Wrist Offset: %f\n", wristHeight, wristOffset);
    }

}
