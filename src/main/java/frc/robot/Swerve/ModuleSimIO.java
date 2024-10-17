package frc.robot.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Swerve.SwerveConstants.SwerveDimensions;

public class ModuleSimIO extends ModuleIO {

    DCMotorSim azimuthMotor;
    DCMotorSim velocityMotor;

    SimpleMotorFeedforward velocityFF;
    SimpleMotorFeedforward angularFF;

    PIDController angleFeedback;
    PIDController velocityFeedback;

    public ModuleSimIO(int id) {
        super(id);

        azimuthMotor = new DCMotorSim(DCMotor.getKrakenX60(1), SwerveDimensions.GEARING, 0.5);
        velocityMotor = new DCMotorSim(DCMotor.getKrakenX60(1), SwerveDimensions.GEARING, 0.5);

        velocityFF = new SimpleMotorFeedforward(1, 1);
        angularFF = new SimpleMotorFeedforward(1, 1);

        angleFeedback = new PIDController(1, 0, 0);
        angleFeedback.enableContinuousInput(-Math.PI, Math.PI);

        velocityFeedback = new PIDController(1, 0, 0);

        azimuthMotor.setState(0, 0);
        velocityMotor.setState(0, 0);
    }

    @Override
    public void setAzimuthVolts(double volts) {
        double output = MathUtil.clamp(volts, -12, 12);
        azimuthMotor.setInputVoltage(output);
    }

    @Override
    public void setLinearVolts(double volts) {
        double output = MathUtil.clamp(volts, -12, 12);
        velocityMotor.setInputVoltage(output);
    }

    @Override
    public void setLinearVelocityMPS(double metersPerSecond) {
        double output = velocityFeedback.calculate(moduleData.linearVelocityMetersPerSec(), metersPerSecond)
                + velocityFF.calculate(metersPerSecond);
        setLinearVolts(output);
    }

    @Override
    public void setAzimuthAngleRPS(double angleRads) {
        double output = angleFeedback.calculate(moduleData.azimuthMotorPos().getRadians(), angleRads)
                + angularFF.calculate(angleFeedback.getVelocityError());
        setAzimuthVolts(output);
    }

    @Override
    public void updateModuleData() {
        moduleData = new ModuleIOData((velocityMotor.getAngularVelocityRPM() * SwerveDimensions.WHEELCIRCUMFERENCE) / 60,
                Rotation2d.fromRadians(azimuthMotor.getAngularVelocityRadPerSec()),
                velocityMotor.getAngularPositionRotations() * SwerveDimensions.WHEELCIRCUMFERENCE,
                Rotation2d.fromRadians(azimuthMotor.getAngularPositionRad()));

    }

}
