package frc.robot.Swerve;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Swerve.GyroIO.GyroIOData;

public class Gyro { 

     GyroIO gyro;  
     GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

     public Gyro(){  
       switch (Constants.getRunType()){ 
          case REAL -> gyro = new GyroPigeonIO();  
          case SIM -> gyro = new GyroSimIO(); 
          case REPLAY -> gyro = new GyroSimIO();
       }
        
     } 

     public GyroIOData getData(){ 
        return gyro.getData();
     } 

     public boolean getIsConnected(){ 
        return gyro.getIsConnected();
     }

     public void update(){   
        gyro.updateGyroData();  
        gyro.updateInputs(inputs);
        Logger.processInputs("Swerve/Gyro",inputs);
     } 

}
