package frc.robot.util.devices;

// Import requried classes
import frc.robot.Constants.CHASSIS;

// Import required libraries
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

public class Gyro extends AHRS {

    private static Gyro instance;

    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro(CHASSIS.GYRO_PORT);
        }
        return instance;
    }

    private Gyro(SerialPort.Port port) {
        super(port);
        this.reset();
    }

    private Gyro(SPI.Port port) {
        super(port);
        this.reset();
    }

    private Gyro(I2C.Port port) {
        super(port);
        this.reset();
    }

    private Gyro() {
        super(SPI.Port.kMXP);
        this.reset();
    }

    /**
     * Returns the total accumulated pitch angle (Y Axis, in degrees) reported by
     * the sensor.
     */
    @Override
    public float getPitch() {
        // Inverts and Rounds pitch angle to 1 decimal
        return (float) Math.round((super.getPitch() * (CHASSIS.GYRO_OUTPUT_INVERTED ? -1 : 1) * 10)) / 10;
    }

}