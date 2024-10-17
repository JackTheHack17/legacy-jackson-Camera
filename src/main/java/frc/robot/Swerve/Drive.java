package frc.robot.Swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Swerve.Module;
import frc.robot.Swerve.SwerveConstants.SwerveSpecifications;

public final class Drive extends SubsystemBase {

    Module[] modules;
    Gyro gyro;
    SwerveDrivePoseEstimator poseEstimator;

    SwerveModulePosition[] lastModulePositions;
    Rotation2d disconnectedHeading = new Rotation2d(0);

    public Drive() {

        modules = new Module[]{
            new Module(1), //FL
            new Module(2), //FR
            new Module(3), //BL
            new Module(4) //BR
        };

        gyro = new Gyro();

        poseEstimator = new SwerveDrivePoseEstimator(SwerveSpecifications.KINEMATICS, getGyroHeading(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
        lastModulePositions = getModulePositions();
    }

    public void runLinearVoltageTests() {
        for (int i = 0; i < 4; i++) {
            modules[i].setLinearVolts(1);
        }
    }

    public void runAzimuthVoltageTests() {
        for (int i = 0; i < 4; i++) {
            modules[i].setAzimuthVolts(1);
        }
    }

    public Rotation2d getGyroHeading() {
        return gyro.getIsConnected() ? gyro.getHeading() : disconnectedHeading;
    }

    public void setSwerveStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        };
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void stop() {
        for (int i = 0; i < 4; i++) {
            modules[i].stop();
        }
    }

    private void updateDisconnectedHeading() {
        SwerveModulePosition[] currentPositions = getModulePositions();
        SwerveModulePosition[] moduleThetas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            moduleThetas[i] = new SwerveModulePosition(
                    currentPositions[i].distanceMeters - lastModulePositions[i].distanceMeters,
                    currentPositions[i].angle
            );
            lastModulePositions[i] = currentPositions[i];
        }

        Twist2d twist = SwerveSpecifications.KINEMATICS.toTwist2d(moduleThetas);
        disconnectedHeading = disconnectedHeading.plus(new Rotation2d(twist.dtheta));
    }

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) {
            modules[i].update();
        }
        gyro.update();
        updateDisconnectedHeading();
        poseEstimator.update(getGyroHeading(), getModulePositions());

    }
}
