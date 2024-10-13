package frc.robot.Swerve.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Swerve.Drive;
import frc.robot.Swerve.SwerveConstants.SwerveSpecifications;

public class SwerveTeleop extends Command {
    DoubleSupplier x; 
    DoubleSupplier y; 
    DoubleSupplier angle;  
    Drive drive;

    public SwerveTeleop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angle, Drive drive){ 
         this.x = x; 
         this.y = y;
         this.angle = angle; 
         this.drive = drive; 
         addRequirements(drive);
    } 

    @Override 
    public void execute(){      
        double xVal = MathUtil.applyDeadband(x.getAsDouble(), 0.1); 
        double yVal = MathUtil.applyDeadband(y.getAsDouble(), 0.1); 
        double angleVal = MathUtil.applyDeadband(angle.getAsDouble(), 0.1); 

        if (xVal + yVal + angleVal == 0){ 
            return;
        } 

        ChassisSpeeds speeds;
        if (!SwerveSpecifications.ISFIELDRELATIVE){ 
            speeds = ChassisSpeeds.fromRobotRelativeSpeeds( 
              xVal * SwerveSpecifications.MAXVELOCITYMPS, 
              yVal * SwerveSpecifications.MAXVELOCITYMPS, 
              angleVal * SwerveSpecifications.MAXVELOCITYRPS, 
              drive.getGyroHeading() 
           );
        } else { 
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds( 
              xVal * SwerveSpecifications.MAXVELOCITYMPS, 
              yVal * SwerveSpecifications.MAXVELOCITYMPS, 
              angleVal * SwerveSpecifications.MAXVELOCITYRPS, 
              drive.getGyroHeading()
           );
        } 

        SwerveModuleState[] states = SwerveSpecifications.KINEMATICS.toSwerveModuleStates(speeds);  
        SwerveDriveKinematics.desaturateWheelSpeeds(SwerveSpecifications.KINEMATICS.toSwerveModuleStates(speeds), SwerveSpecifications.MAXVELOCITYMPS);
        drive.setSwerveStates(states);
    } 

    @Override 
    public boolean isFinished(){  
        return true;
    }  

    @Override 
    public void end(boolean interrupted){ 
        drive.stop();
    }
}
