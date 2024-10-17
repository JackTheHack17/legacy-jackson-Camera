package frc.robot.Swerve;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Swerve.ModuleIO.ModuleIOData;


public class Module {  

    ModuleIO module;
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    
    public Module(int id){   
       switch (Constants.getRunType()){ 
         case SIM -> module = new ModuleSimIO(id);
         case REAL -> module = new ModuleTalonIO(id); 
         case REPLAY -> module = new ModuleSimIO(id);
       }
    }  

    public void setDesiredState(SwerveModuleState state){  
       module.setDesiredState(state);
    }  

    public SwerveModulePosition getPosition(){ 
       return module.getPosition();
    }

    public void setAzimuthVolts(double volts){ 
       module.setAzimuthVolts(volts);
    } 

    public void setLinearVolts(double volts){ 
       module.setLinearVolts(volts);
    }   

    public void stop(){ 
       module.stop();
    } 

    public void update(){  
       module.updateModuleData();  
       module.updateInputs(inputs);
       Logger.processInputs(module.getKey(),inputs);
    } 

    public ModuleIOData getData(){ 
       return module.getData();
    }

}
