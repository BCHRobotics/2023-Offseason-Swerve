package frc.robot.util.control;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkMaxConstants {

    // PID coefficients
    public double kP;
    public double kI;
    public double kD;
    public double kIz;
    public double kFF;
    public double kMinOutput;
    public double kMaxOutput;

    // Smart Motion Coefficients
    public int slot;
    public double minVel; // rpm
    public double maxVel; // rpm
    public double maxAcc; // rpm^2
    public double allowedErr;

    public String name;

    /**
     * PID Values for Spark MAX Controller
     * 
     * @param kP
     * @param kI
     * @param kD
     * @param kIz
     * @param kFF
     * @param kMinOutput
     * @param kMaxOutput
     * @param slot
     * @param minVel
     * @param maxVel
     * @param maxAcc
     * @param allowedErr
     */
    public SparkMaxConstants(double kP, double kI, double kD, double kIz, double kFF, double kMinOutput,
            double kMaxOutput, int slot, double minVel, double maxVel, double maxAcc, double allowedErr) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kIz = kIz;
        this.kFF = kFF;
        this.kMinOutput = kMinOutput;
        this.kMaxOutput = kMaxOutput;
        this.slot = slot;
        this.minVel = minVel;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
        this.allowedErr = allowedErr;
    }

    public void pushToDashboard(String name) {
        this.name = name;
        SmartDashboard.putNumber(this.name + " P Gain", kP);
        SmartDashboard.putNumber(this.name + " I Gain", kI);
        SmartDashboard.putNumber(this.name + " D Gain", kD);
        SmartDashboard.putNumber(this.name + " Feed Forward", kFF);
        SmartDashboard.putNumber(this.name + " I Zone", kIz);
        SmartDashboard.putNumber(this.name + " Max Output", kMaxOutput);
        SmartDashboard.putNumber(this.name + " Min Output", kMinOutput);
        SmartDashboard.putNumber(this.name + " Min Velocity", minVel);
        SmartDashboard.putNumber(this.name + " Max Velocity", maxVel);
        SmartDashboard.putNumber(this.name + " Max Acceleration", maxAcc);
        SmartDashboard.putNumber(this.name + " Allowed Closed Loop Error", allowedErr);
    }

    public void getFromDashboard() {
        kP = SmartDashboard.getNumber(this.name + " P Gain", 0);
        kI = SmartDashboard.getNumber(this.name + " I Gain", 0);
        kD = SmartDashboard.getNumber(this.name + " D Gain", 0);
        kFF = SmartDashboard.getNumber(this.name + " Feed Forward", 0);
        kIz = SmartDashboard.getNumber(this.name + " I Zone", 0);
        kMinOutput = SmartDashboard.getNumber(this.name + " Min Output", 0);
        kMaxOutput = SmartDashboard.getNumber(this.name + " Max Output", 0);
        minVel = SmartDashboard.getNumber(this.name + " Min Velocity", 0);
        maxVel = SmartDashboard.getNumber(this.name + " Max Velocity", 0);
        maxAcc = SmartDashboard.getNumber(this.name + " Max Acceleration", 0);
        allowedErr = SmartDashboard.getNumber(this.name + " Allowed Closed Loop Error", 0);
    }

    public boolean valuesChanged() {
        return (this.kP != SmartDashboard.getNumber(this.name + " P Gain", 0)
                || this.kI != SmartDashboard.getNumber(this.name + " I Gain", 0)
                || this.kD != SmartDashboard.getNumber(this.name + " D Gain", 0)
                || this.kFF != SmartDashboard.getNumber(this.name + " Feed Forward", 0)
                || this.kIz != SmartDashboard.getNumber(this.name + " I Zone", 0)
                || this.kMinOutput != SmartDashboard.getNumber(this.name + " Min Output", 0)
                || this.kMaxOutput != SmartDashboard.getNumber(this.name + " Max Output", 0)
                || this.minVel != SmartDashboard.getNumber(this.name + " Min Velocity", 0)
                || this.maxVel != SmartDashboard.getNumber(this.name + " Max Velocity", 0)
                || this.maxAcc != SmartDashboard.getNumber(this.name + " Max Acceleration", 0)
                || this.allowedErr != SmartDashboard.getNumber(this.name + " Allowed Closed Loop Error", 0));
    }

    @Override
    public String toString() {
        return String.format("kP: %f, kI: %f, kD: %f, kIz: %f, kFF: %f\n" +
                "kMaxOutput: %f, kMinOutput: %f\n" +
                "maxVel: %f, minVel: %f, maxAcc: %f, allowedErr: %f",
                kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr);
    }
}
