package frc.robot.Swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Swerve.SwerveConstants.SwerveDimensions;
import frc.robot.Swerve.SwerveConstants.SwerveMotorSafety;
import frc.robot.Swerve.SwerveConstants.TalonModuleSpecifications;

public class ModuleTalonIO extends ModuleIO {
     
    TalonModuleSpecifications moduleConstants; 

    TalonFX speedMotor;  
    TalonFX azimuthMotor; 
    CANcoder absoluteEncoder;
    Rotation2d angleOffset;

    public ModuleTalonIO(int id) {
        super(id); 

        switch (key) { 
          case "FL"-> moduleConstants = SwerveConstants.frontLeft; 
          case "FR" -> moduleConstants = SwerveConstants.frontRight; 
          case "BL" -> moduleConstants = SwerveConstants.backLeft; 
          case "BR" -> moduleConstants = SwerveConstants.backRight; 
        }    

        speedMotor = new TalonFX(moduleConstants.speedMotorPort()); 
        azimuthMotor = new TalonFX(moduleConstants.azimuthMotorPort());    
        absoluteEncoder = new CANcoder(moduleConstants.absEncoderPort());  
        angleOffset = moduleConstants.angleOffset(); 
        
        configureHardware();
        
    } 

    private void configureHardware(){  
        speedMotor.setSafetyEnabled(true); 
        azimuthMotor.setSafetyEnabled(true); 

        speedMotor.setInverted(moduleConstants.isSpeedInverted()); 
        azimuthMotor.setInverted(moduleConstants.isAzimuthInverted()); 
        
        speedMotor.setNeutralMode(SwerveMotorSafety.NEUTRALMODE); 
        azimuthMotor.setNeutralMode(SwerveMotorSafety.NEUTRALMODE); 
        //-----------------------VELOCITY----------------------------
        TalonFXConfiguration speedConfig = new TalonFXConfiguration();   
        
        speedConfig.CurrentLimits = getCurrentConfig(); 
        speedConfig.Slot0 = getSpeedSlotConfigs();
    
        speedMotor.getConfigurator().apply(speedConfig);   
        //-----------------------AZIMUTH---------------------------- 
        TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();  

        azimuthConfig.CurrentLimits = getCurrentConfig();  
        azimuthConfig.Slot0 = getAzimuthSlotConfigs();

        azimuthMotor.getConfigurator().apply(azimuthConfig);  
        //----------------------Encoder Configuration------------------- 

        speedMotor.setPosition(0); 
        azimuthMotor.setPosition(getAbsAzimuthPosRotations());
    } 

    private CurrentLimitsConfigs getCurrentConfig(){ 
        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();  
        currentConfig.StatorCurrentLimitEnable = true; 
        currentConfig.SupplyCurrentLimitEnable = true;  
        currentConfig.StatorCurrentLimit = SwerveMotorSafety.STATORCURRENTLIMIT; 
        currentConfig.SupplyCurrentLimit = SwerveMotorSafety.SUPPLYCURRENTLIMIT;  
        return currentConfig;
    }    

    private Slot0Configs getAzimuthSlotConfigs(){ 
        Slot0Configs currentConfig = new Slot0Configs(); 
        currentConfig.kP = 1; //PID: P
        currentConfig.kI = 1; //PID: I
        currentConfig.kD = 1; //PID: D

        currentConfig.kS = 1; //FF: S
        currentConfig.kV = 1; //FF: V 
        return currentConfig;
    }  

    private Slot0Configs getSpeedSlotConfigs(){ 
        Slot0Configs currentConfig = new Slot0Configs(); 
        currentConfig.kP = 1; //PID: P
        currentConfig.kI = 1; //PID: I
        currentConfig.kD = 1; //PID: D

        currentConfig.kS = 1; //FF: S
        currentConfig.kV = 1; //FF: V 
        return currentConfig;
    } 


    private double getAbsAzimuthPosRotations(){ 
        return (absoluteEncoder.getPosition().getValueAsDouble() - angleOffset.getRotations());
    }


    @Override
    public void setAzimuthVolts(double volts) {
        azimuthMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setLinearVolts(double volts) {
        speedMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setLinearVelocityMPS(double metersPerSecond) {
        speedMotor.setControl(new VelocityVoltage(metersPerSecond / (SwerveDimensions.GEARING * SwerveDimensions.WHEELCIRCUMFERENCE))); 
    }

    @Override
    public void setAzimuthAngleRPS(double angleRads) {
        speedMotor.setControl(new PositionVoltage(Units.radiansToRotations(angleRads) * SwerveDimensions.GEARING)); 
    }

    @Override
    public void updateModuleData() { 
        moduleData = new ModuleIOData(speedMotor.getVelocity().getValueAsDouble() * SwerveDimensions.GEARING * SwerveDimensions.WHEELCIRCUMFERENCE, 
                                      Rotation2d.fromRotations(azimuthMotor.getVelocity().getValueAsDouble()), 
                                      speedMotor.getPosition().getValueAsDouble() * SwerveDimensions.GEARING * SwerveDimensions.WHEELCIRCUMFERENCE, 
                                      Rotation2d.fromRotations(azimuthMotor.getPosition().getValueAsDouble() * SwerveDimensions.GEARING)
                                      ); 

        
    }
    
}
