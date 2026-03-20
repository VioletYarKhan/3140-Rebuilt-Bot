// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.libs.Vector2;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;

import org.littletonrobotics.junction.Logger;

public class PoseOdometry extends Odometry {
  Pose2d simStartingPose = new Pose2d(14, 2, new Rotation2d(Units.degreesToRadians(75)));

  protected SwerveDrivePoseEstimator estimator = null;
  protected SwerveDrivePoseEstimator simEstimator = null;
  protected SwerveDrivePoseEstimator simDriftEstimator = null;
  private LinearFilter velXFilter = LinearFilter.movingAverage(4);
  private LinearFilter velYFilter = LinearFilter.movingAverage(4);
  private LinearFilter rotVelFilter = LinearFilter.movingAverage(4);
  private Vector2 smoothedVelocity = new Vector2();
  private double smoothedRotVel = 0;
  private boolean knowsPosition = false;
  private Pose2d nullPose = new Pose2d(0, 0, new Rotation2d(0));

  private Pose2d startingPose = null;
  private final int startingCameraPasses = Constants.Odometry.startingCameraPasses;
  private int cameraPasses = 0;
  private double angleOffset = 0;

  protected PoseOdometry() {
    super();
    NavXSim.getInstance().reset(simStartingPose.getRotation().getRadians());
  }

  public double getX() {
    return estimator == null ? 0 : estimator.getEstimatedPosition().getX();
  }

  public double getY() {
    return estimator == null ? 0 : estimator.getEstimatedPosition().getY();
  }

  public double getAngle() {
    return getRotation().getRadians();
  }

  public Rotation2d getRotation() {
    return estimator == null ? new Rotation2d() : estimator.getEstimatedPosition().getRotation();
  }

  public Vector2 getPosition() {
    return new Vector2(getX(), getY());
  }

  public boolean knowsPose() {
    return knowsPosition;
  }

  @Override
  public void update() {
    super.update();
  }

  @Override
  public void resetPose(Pose2d pose) {
    if (estimator == null) {
      estimator = new SwerveDrivePoseEstimator(
          SwerveDrive.getInstance().kinematics,
          pose.getRotation(),
          SwerveDrive.getInstance().getModulePositions(),
          pose);
    } else {
      estimator.resetPose(pose);
    }

    if (RobotBase.isSimulation()) {
      NavXSim.getInstance().reset(pose.getRotation().getRadians());

      // Reset all the module encoders to 0
      for (SwerveModule module : SwerveDrive.getInstance().modules) {
        module.simDriveMotor.setPosition(0);

        module.simTurnMotor.setPosition(0);
      }
    }

    System.out.println("[Odometry] Reset Pose to " + pose);
  }

  @Override
  public Pose2d getPose() {
    return estimator == null ? nullPose : estimator.getEstimatedPosition();
  }

  @Override
  public Pose2d getRealSimPose() {
    return simEstimator == null ? nullPose : simEstimator.getEstimatedPosition();
  }

  @Override
  public Rotation2d getGyroRotation() {
    return new Rotation2d(
        (!RobotBase.isSimulation() ? gyro.getRotation2d().getRadians()
            : NavXSim.getInstance().getRotation2d().getRadians())
            + angleOffset);
  }

  @Override
  public void periodic() {
    super.periodic();
    ChassisSpeeds field = SwerveDrive.getInstance().getFieldRelativeSpeeds();
    Logger.recordOutput("ChassisSpeeds/Field/vx", field.vxMetersPerSecond);
    Logger.recordOutput("ChassisSpeeds/Field/vy", field.vyMetersPerSecond);
    Logger.recordOutput("ChassisSpeeds/Field/omega", field.omegaRadiansPerSecond);

    ChassisSpeeds accel = SwerveDrive.getInstance().getFieldRelativeAcceleration();
    Logger.recordOutput("Accel/Field/X", accel.vxMetersPerSecond);
    Logger.recordOutput("Accel/Field/Y", accel.vyMetersPerSecond);
    Logger.recordOutput("Accel/Field/Angular", accel.omegaRadiansPerSecond);

    Vector2 rawVel = getRawBotVelocity();
    smoothedVelocity = new Vector2(velXFilter.calculate(rawVel.X), velYFilter.calculate(rawVel.Y));
    smoothedRotVel = rotVelFilter.calculate(getRawAngularVelocity());

    Logger.recordOutput("Odometry/speed/X", getBotVelocity(true).X);
    Logger.recordOutput("Odometry/speed/Y", getBotVelocity(true).Y);
    Logger.recordOutput("Odometry/speed/magnitude", getBotVelocity(true).magnitude());
    Logger.recordOutput("Odometry/rotSpeed", smoothedRotVel);
    Logger.recordOutput("Odometry/rotSpeedRaw", getRawAngularVelocity());
  }

  public void resetGyro() {
    NavXSim.getInstance().reset(0);
    resetGyroCamera(0);
  }

  public void resetGyroCamera(double correctAngle) {
    angleOffset = -readRotationRaw() + correctAngle;

  }

  public void recalibrateCameraPose() {
    cameraPasses = 0;
  }

  @Override
  public void updatePosition(SwerveModulePosition[] positions) {
    SwerveDrive drive = SwerveDrive.getInstance();
    double stdDev = 2.5;
    if (estimator == null) {
      estimator = new SwerveDrivePoseEstimator(
          drive.kinematics,
          getGyroRotation(),
          positions,
          new Pose2d());
      estimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDev, stdDev, Units.degreesToRadians(15)));

      if (RobotBase.isSimulation()) {
        simEstimator = new SwerveDrivePoseEstimator(
            drive.kinematics,
            getGyroRotation(),
            positions,
            simStartingPose);
        simDriftEstimator = new SwerveDrivePoseEstimator(
            drive.kinematics,
            getGyroRotation(),
            positions,
            simStartingPose);
        simDriftEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDev, stdDev, Units.degreesToRadians(15)));
      }
    }

    if (RobotBase.isSimulation()) {
      simEstimator.update(getGyroRotation(), positions);
      positions[0].distanceMeters *= 1.03;
      positions[1].distanceMeters *= 1.03;
      positions[2].distanceMeters *= 1.03;
      positions[3].distanceMeters *= 1.03;
      simDriftEstimator.update(getGyroRotation(), positions);
      Logger.recordOutput("Odometry/simRealPosition", simEstimator.getEstimatedPosition());
      Logger.recordOutput("Odometry/realisticOdometryBot", simDriftEstimator.getEstimatedPosition());
    }
    estimator.update(getGyroRotation(), positions);

    Logger.recordOutput("Odometry/simVisionBot", estimator.getEstimatedPosition());
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    if (estimator != null) {

      if (cameraPasses == 0) {
        startingPose = null;
        cameraPasses++;
      } else if (cameraPasses < startingCameraPasses) {
        if (pose != null) {
          if (startingPose == null)
            startingPose = pose;
          startingPose = startingPose.interpolate(pose, 1.0 / startingCameraPasses);
          cameraPasses++;
        }
      } else if (cameraPasses == startingCameraPasses) {
        estimator.resetPose(startingPose);
        resetGyroCamera(startingPose.getRotation().getRadians());
        cameraPasses++;
      } else {
        if (estimator.getEstimatedPosition().getTranslation()
            .getDistance(pose.getTranslation()) < Constants.Odometry.maxCorrectionDistance) {
          estimator.addVisionMeasurement(
              pose,
              Timer.getFPGATimestamp());
        }
        estimator.resetRotation(getGyroRotation());
      }
    }
  }

  @Override
  public double getAngularVelocity() {
    return SwerveDrive.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
  }

  public double getRawAngularVelocity() {
    return smoothedRotVel;
  }

  public Vector2 getBotVelocity(boolean fieldRelative) {

    return fieldRelative ? smoothedVelocity.rotate(getAngle()) : smoothedVelocity;
  }

  public Vector2 getRawBotVelocity() {
    return new Vector2(SwerveDrive.getInstance().getRobotRelativeSpeeds().vxMetersPerSecond,
        SwerveDrive.getInstance().getRobotRelativeSpeeds().vyMetersPerSecond);
  }

  public Vector2 getBotAcceleration() {
    return new Vector2();

    // return new
    // Vector2(SwerveDrive.getInstance().getFieldRelativeAcceleration().vxMetersPerSecond,
    // SwerveDrive.getInstance().getFieldRelativeAcceleration().vyMetersPerSecond);
  }

  @Override
  public void updateSimulatedPosition(SwerveModulePosition[] positions, double gyroAngleRad) {
  }
}
