package frc.robot.Swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RunType;


public class SwerveConstants {  
    
    // To-do 
    // Plug in values for these constants 
    
    public record TalonModuleSpecifications(int speedMotorPort, int azimuthMotorPort, int absEncoderPort, boolean isSpeedInverted, boolean isAzimuthInverted, Rotation2d angleOffset){};
    
    public static TalonModuleSpecifications frontLeft = new TalonModuleSpecifications(0,0,0,false, false, new Rotation2d(0));
    public static TalonModuleSpecifications frontRight = new TalonModuleSpecifications(0,0,0,false,false, new Rotation2d(0)); 
    public static TalonModuleSpecifications backLeft = new TalonModuleSpecifications(0,0,0,false,false, new Rotation2d(0)); 
    public static TalonModuleSpecifications backRight = new TalonModuleSpecifications(0,0,0,false,false, new Rotation2d(0)); 


    public enum ModuleType { 
       SIM, 
       TALON
    }; 

    public class SwerveSpecifications {  
        public static boolean ISFIELDRELATIVE = false;  

        public static final double MAXVELOCITYMPS = 20; 
        public static final double MAXVELOCITYRPS = 5; 

        public static final ModuleType CURRENT_MODUL_TYPE = frc.robot.Constants.getRunType() == RunType.SIM ? ModuleType.SIM : ModuleType.TALON;   

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d[]{ 
                new Translation2d(SwerveDimensions.TRACKWIDTHXMETERS / 2,SwerveDimensions.TRACKWIDTHYMETERS / 2), //FL
                new Translation2d(SwerveDimensions.TRACKWIDTHXMETERS / 2,SwerveDimensions.TRACKWIDTHYMETERS / 2), //FR
                new Translation2d(SwerveDimensions.TRACKWIDTHXMETERS / 2,SwerveDimensions.TRACKWIDTHYMETERS / 2), //BL
                new Translation2d(SwerveDimensions.TRACKWIDTHXMETERS / 2,SwerveDimensions.TRACKWIDTHYMETERS / 2)  //BR
            });   
    }  


    public class SwerveDimensions { 
       public static final double TRACKWIDTHYMETERS = 5;  
       public static final double TRACKWIDTHXMETERS = 3;
       public static final double WHEELBASE = 3;  
       public static final double GEARING = 7;   

       private static final double WHEELRADIUS = Units.inchesToMeters(3);
       public static final double WHEELCIRCUMFERENCE = WHEELRADIUS * 2 * Math.PI;
    }  


    public class SwerveMotorSafety { 
          public static NeutralModeValue NEUTRALMODE = NeutralModeValue.Coast; 
          public static double STATORCURRENTLIMIT = 60; 
          public static double SUPPLYCURRENTLIMIT = 80;
    }


}
