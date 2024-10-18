package frc.robot.Swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class GyroIO {

    public boolean isConnected = false;

    @AutoLog
    public static class GyroIOInputs {

        public Rotation2d yaw = new Rotation2d();
        public Rotation2d pitch = new Rotation2d();
        public Rotation2d roll = new Rotation2d();
    }

    public record GyroIOData(Rotation2d yaw, Rotation2d pitch, Rotation2d roll) {

    }

    protected GyroIOData gyroData = new GyroIOData(new Rotation2d(), new Rotation2d(), new Rotation2d());

    abstract void updateGyroData();  //Updates record

    public GyroIOData getData() {
        return gyroData;
    }

    public void setIsConencted(boolean connected) {
        isConnected = connected;
    }

    public boolean getIsConnected() {
        return isConnected;
    }

    public void updateInputs(GyroIO.GyroIOInputs inputs) {
        inputs.pitch = gyroData.pitch();
        inputs.roll = gyroData.roll();
        inputs.yaw = gyroData.yaw();
    }

}
