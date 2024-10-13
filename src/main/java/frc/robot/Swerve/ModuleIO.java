package frc.robot.Swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public abstract class ModuleIO {
    
    @AutoLog  
    public class ModuleIOInputs { 
        public double linearVelocityMetersPerSec = 0; 
        public Rotation2d angularVelocityPerSec = new Rotation2d(0); 
        public double linearMotorPosMeters = 0; 
        public Rotation2d azimuthMotorPos = new Rotation2d(0);  
    }  
    public record ModuleIOData(double linearVelocityMetersPerSec, Rotation2d angularVelocityPerSec, double linearMotorPosMeters, Rotation2d azimuthMotorPos){}

    protected String key; 
    protected int id;       
    protected ModuleIOData moduleData;

    public ModuleIO(int id){  
        this.id = id; 
        switch (id){  
           case 1 -> key = "FL"; 
           case 2 -> key = "FR"; 
           case 3 -> key = "BL"; 
           case 4 -> key = "BR";
        } 
        moduleData = new ModuleIOData(0, new Rotation2d(), 0 ,new Rotation2d());
    }

    public abstract void setAzimuthVolts(double volts); 

    public abstract void setLinearVolts(double volts); 
    
    public abstract void setLinearVelocityMPS(double metersPerSecond); 

    public abstract void setAzimuthAngleRPS(double angleRads);   

    public abstract void updateModuleData(); //Updates the record
    
    public String getKey(){ 
       return "Swerve/Module/".concat(key);
    }  

    public SwerveModulePosition getPosition(){ 
      return new SwerveModulePosition(moduleData.linearMotorPosMeters(), moduleData.azimuthMotorPos());
    } 

    public void stop(){ 
        setLinearVolts(0); 
        setAzimuthVolts(0);
    }

    public void setDesiredState(SwerveModuleState moduleState){ 
        setLinearVelocityMPS(moduleState.speedMetersPerSecond); 
        setAzimuthAngleRPS(moduleState.angle.getRadians());
    }  

    public void updateInputs(ModuleIOInputs inputs){ 
         inputs.angularVelocityPerSec = moduleData.angularVelocityPerSec(); 
         inputs.azimuthMotorPos = moduleData.azimuthMotorPos(); 
         inputs.linearMotorPosMeters = moduleData.linearMotorPosMeters(); 
         inputs.linearVelocityMetersPerSec = moduleData.linearVelocityMetersPerSec();
    } 

    public ModuleIOData getData(){ 
        return moduleData;
    }

    
}
