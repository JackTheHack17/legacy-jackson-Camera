package frc.robot.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class Gyro {

    GyroIO gyro;
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public Gyro() {
        switch (Constants.getRunType()) {
            case REAL ->
                gyro = new GyroPigeonIO();
            case SIM ->
                gyro = new GyroSimIO();
            case REPLAY ->
                gyro = new GyroSimIO();
        }
    }

    public Rotation2d getHeading() {
        return gyro.getData().yaw();
    }

    public boolean getIsConnected() {
        return gyro.getIsConnected();
    }

    public void update() {
        gyro.updateGyroData();
        gyro.updateInputs(inputs);
        Logger.processInputs("Swerve/Gyro", inputs);
    }

}
