package frc.robot.Swerve;

import org.littletonrobotics.junction.Logger;

import frc.robot.Swerve.GyroIO.GyroIOData;

public class Gyro { 

     GyroIO gyro;  
     GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

     public Gyro(){ 
        gyro = new GyroPigeonIO();
     } 

     public GyroIOData getData(){ 
        return gyro.getData();
     }

     public void update(){   
        gyro.updateGyroData();  
        gyro.updateInputs(inputs);
        Logger.processInputs("Swerve/Gyro",inputs);
        
     } 

}
