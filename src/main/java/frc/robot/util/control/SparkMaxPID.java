package frc.robot.util.control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import frc.robot.Constants.Misc;

public class SparkMaxPID {

    private SparkMaxPIDController pidController;
    private SparkMaxConstants constants;
    private double setpoint;

    public SparkMaxPID(CANSparkMax motor) {
        pidController = motor.getPIDController();
    }

    public SparkMaxPID(CANSparkMax motor, SparkMaxConstants constants) {
        this(motor);
        this.setConstants(constants);
    }

    public SparkMaxPID(SparkMaxPIDController pidController) {
        this.pidController = pidController;
    }

    public SparkMaxPID(SparkMaxPIDController pidController, SparkMaxConstants constants) {
        this(pidController);
        this.setConstants(constants);
    }

    public void setConstants(SparkMaxConstants c) {
        this.pidController.setP(c.kP, c.slot);
        this.pidController.setI(c.kI, c.slot);
        this.pidController.setD(c.kD, c.slot);
        this.pidController.setFF(c.kFF, c.slot);
        this.pidController.setIZone(c.kIz, c.slot);
        this.pidController.setOutputRange(c.kMinOutput, c.kMaxOutput, c.slot);
        this.constants = c;
        this.pidController.setSmartMotionMinOutputVelocity(c.minVel, c.slot);
        this.pidController.setSmartMotionMaxVelocity(c.maxVel, c.slot);
        this.pidController.setSmartMotionMaxAccel(c.maxAcc, c.slot);
        this.pidController.setSmartMotionAllowedClosedLoopError(c.allowedErr, c.slot);
    }

    public SparkMaxConstants getRawConstants(int slot) {
        return new SparkMaxConstants(
                this.pidController.getP(slot),
                this.pidController.getI(slot),
                this.pidController.getD(slot),
                this.pidController.getFF(slot),
                this.pidController.getIZone(slot),
                this.pidController.getOutputMin(slot),
                this.pidController.getOutputMax(slot),
                this.constants.slot,
                this.pidController.getSmartMotionMinOutputVelocity(slot),
                this.pidController.getSmartMotionMaxVelocity(slot),
                this.pidController.getSmartMotionMaxAccel(slot),
                this.pidController.getSmartMotionAllowedClosedLoopError(slot));
    }

    public SparkMaxConstants getRawConstants() {
        return this.getRawConstants(this.constants.slot);
    }

    public SparkMaxConstants getConstants() {
        return this.constants;
    }

    public void setFeedForward(double inputFF) {
        this.constants.kFF = inputFF;
        this.setConstants(this.constants);
    }

    public void pushConstantsToDashboard(String label) {
        this.constants.pushToDashboard(label);
    }

    public void retrieveDashboardConstants() {
        if (this.constants.valuesChanged()) {
            this.constants.getFromDashboard();
            this.setConstants(constants);
        }
    }

    public boolean constantsChanged() {
        return this.constants.valuesChanged();
    }

    public void setFeedbackDevice(RelativeEncoder device) {
        this.pidController.setFeedbackDevice(device);
    }

    public void setFeedbackDevice(SparkMaxAbsoluteEncoder device) {
        this.pidController.setFeedbackDevice(device);
    }

    public void setFeedbackDevice(SparkMaxAlternateEncoder device) {
        this.pidController.setFeedbackDevice(device);
    }

    public void setPIDWrapping(boolean state) {
        this.pidController.setPositionPIDWrappingEnabled(state);
        this.pidController.setPositionPIDWrappingMinInput(0);
        this.pidController.setPositionPIDWrappingMaxInput(360);
    }

    public void setMotionProfileType(AccelStrategy strategy) {
        this.pidController.setSmartMotionAccelStrategy(strategy, this.constants.slot);
    }

    public void setSmartPosition(double position) {
        this.setpoint = position;
        this.pidController.setReference(position, CANSparkMax.ControlType.kSmartMotion, this.constants.slot);
    }

    public void setSmartPosition(double position, double min, double max) {
        this.setpoint = position;
        this.setSmartPosition(Misc.ENSURE_RANGE(position, min, max));
    }

    public void setSmartVelocity(double velocity) {
        this.setpoint = velocity;
        this.pidController.setReference(velocity, CANSparkMax.ControlType.kSmartVelocity, this.constants.slot);
    }

    public void setPosition(double position) {
        this.setpoint = position;
        this.pidController.setReference(position, CANSparkMax.ControlType.kPosition, this.constants.slot);
    }

    public void setPosition(double position, double min, double max) {
        this.setpoint = position;
        this.setPosition(Misc.ENSURE_RANGE(position, min, max));
    }

    public void setVelocity(double speed) {
        this.setpoint = speed;
        this.pidController.setReference(speed, CANSparkMax.ControlType.kVelocity, this.constants.slot);
    }

    public double getSetpoint() {
        return this.setpoint;
    }

    public boolean reachedSetpoint(double input, double tolerance) {
        return Misc.WITHIN_TOLERANCE(input, this.setpoint, tolerance);
    }

}
