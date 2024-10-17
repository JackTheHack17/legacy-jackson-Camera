package frc.robot.Swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroPigeonIO extends GyroIO { 

    Pigeon2 hardwareGyro; 
    StatusSignal<Double> yawSupplier; 
    StatusSignal<Double> pitchSupplier; 
    StatusSignal<Double> rollSupplier;

    public GyroPigeonIO(){ 
        hardwareGyro = new Pigeon2(0); //Plug in accordingly  
        hardwareGyro.getConfigurator().apply(new Pigeon2Configuration()); 
        hardwareGyro.getConfigurator().setYaw(0);  

        yawSupplier = hardwareGyro.getYaw(); 
        pitchSupplier = hardwareGyro.getPitch(); 
        rollSupplier = hardwareGyro.getRoll(); 

        setIsConencted(true);
    } 

    @Override
    void updateGyroData() {
       gyroData = new GyroIOData(Rotation2d.fromDegrees(yawSupplier.getValueAsDouble()), 
                                 Rotation2d.fromDegrees(pitchSupplier.getValueAsDouble()),  
                                 Rotation2d.fromDegrees(rollSupplier.getValueAsDouble()));
        
    }
    
}
