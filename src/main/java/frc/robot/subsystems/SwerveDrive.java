// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.NetworkTables;
import frc.robot.subsystems.Turret.TurretMain;
import frc.robot.subsystems.Turret.TurretMain.LoggedPIDInputs;
import frc.robot.subsystems.odometry.Odometry;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive extends SubsystemBase {

  private static SwerveDrive instance = SwerveDrive.getInstance();
  PIDController thetaController = new PIDController(5, 0, 0);
  LoggedPIDInputs thetaPIDInputs = new LoggedPIDInputs("ThetaController", 1.5, 0, 0);
  SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
  // private Camera camera = Camera.getInstance();
  public static Odometry odometry;

  // Locations of the swerve modules relative to the robot center
  public final Translation2d[] locations = {
      // Order: FR, FL, BR, BL --> Necessary order for Custom Dashboard visualization!
      // WPILib uses x = forward, y = left, so front-left is (+x/2, +y/2)
      new Translation2d(Constants.Bot.botLength / 2, -Constants.Bot.botLength / 2),
      new Translation2d(Constants.Bot.botLength / 2, Constants.Bot.botLength / 2),
      new Translation2d(-Constants.Bot.botLength / 2, -Constants.Bot.botLength / 2),
      new Translation2d(-Constants.Bot.botLength / 2, Constants.Bot.botLength / 2)
  };

  public final SwerveModule[] modules = {
      new SwerveModule(
          "frontRight",
          Constants.SensorIDs.FR,
          Constants.MotorIDs.FRVortex,
          Constants.MotorIDs.FRNeo,
          Constants.Bot.FRZeroOffset,
          true),
      new SwerveModule(
          "frontLeft",
          Constants.SensorIDs.FL,
          Constants.MotorIDs.FLVortex,
          Constants.MotorIDs.FLNeo,
          Constants.Bot.FLZeroOffset,
          true),
      new SwerveModule("backRight",
          Constants.SensorIDs.BR,
          Constants.MotorIDs.BRVortex,
          Constants.MotorIDs.BRNeo,
          Constants.Bot.BRZeroOffset,
          true),
      new SwerveModule("backLeft",
          Constants.SensorIDs.BL,
          Constants.MotorIDs.BLVortex,
          Constants.MotorIDs.BLNeo,
          Constants.Bot.BLZeroOffset,
          false)
  };

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      locations[0], locations[1], locations[2], locations[3]);

  public static SwerveDrive getInstance() {
    if (instance == null) {
      instance = new SwerveDrive();
    }
    return instance;
  }

  private SwerveDrive() {

    thetaController.setPID(thetaPIDInputs.getP(), thetaPIDInputs.getI(), thetaPIDInputs.getD());

    NetworkTables.lookTowardsTarget_b.setBoolean(false);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.PI / 45); // 4 degrees
    odometry = Odometry.getInstance();

    NetworkTables.maxVelo.setDouble(Constants.Bot.maxChassisSpeed);

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants( // Translation PID constants
                Constants.PathplannerConstants.TransP,
                Constants.PathplannerConstants.TransI,
                Constants.PathplannerConstants.TransD),
            new PIDConstants( // Rotation PID constants
                Constants.PathplannerConstants.RotP,
                Constants.PathplannerConstants.RotI,
                Constants.PathplannerConstants.RotD)),
        config, // The robot configuration
        this::shouldFlipPath,
        this // Reference to this subsystem to set requirements
    );
  }

  public void periodic() {
    thetaController.setPID(thetaPIDInputs.getP(), thetaPIDInputs.getI(), thetaPIDInputs.getD());
    thetaPIDInputs.update(TurretMain.getInstance().getLookDirection(),
        ((odometry.getGyroRotation().getDegrees() + 180) % 360 + 360) % 360);
    odometry.update();
    updateNetworktables();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean lookAtTurretTarget) {
    if (RobotBase.isSimulation()) {
      rot *= -1;
      ySpeed *= -1;
      xSpeed *= -1;
    }

    if (!fieldRelative && lookAtTurretTarget) {
      System.err.println("lookAtTurretTarget will not work when fieldRelative is false. (SwerveDrive.drive())");
    }

    ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, rot), .02);
    swerveModuleStates = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,
                    lookAtTurretTarget
                        ? thetaController.calculate(odometry.getGyroRotation().getRadians()+Math.PI,
                            Units.degreesToRadians(TurretMain.getInstance().getLookDirection()))
                        : rot,
                    odometry.getGyroRotation()
                        .plus(!RobotBase.isSimulation() ? new Rotation2d(Math.PI) : new Rotation2d(0)))
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            .02));

    // System.out.println(TurretMain.getInstance().getLookDirection() + "\t" +
    // Odometry.getInstance().getGyroRotation().getDegrees());

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Bot.maxChassisSpeed);

    for (int i = 0; i < 4; i++) {
      modules[i].setStates(swerveModuleStates[i]);
    }
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    drive(xSpeed, ySpeed, rot, fieldRelative, false);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  public boolean shouldFlipPath() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      double distanceMeters = modules[i].driveEncoder.getPosition() * Constants.Bot.encoderRotationToMeters;
      Rotation2d angle = Rotation2d.fromDegrees(modules[i].getTurnEncoder().getAbsolutePosition());
      positions[i] = new SwerveModulePosition(distanceMeters, angle);
    }
    return positions;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getRobotRelativeSpeeds(),
        odometry.getRotation());
  }

  private final LinearFilter xAccelFilter = LinearFilter
      .movingAverage(5);
  private final LinearFilter yAccelFilter = LinearFilter
      .movingAverage(5);
  private ChassisSpeeds lastFieldSpeeds = new ChassisSpeeds();
  private double lastTimestamp = Timer.getFPGATimestamp();

  public ChassisSpeeds getFieldRelativeAcceleration() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;

    if (dt <= 1e-6) {
      return new ChassisSpeeds();
    }

    ChassisSpeeds current = getFieldRelativeSpeeds();

    double rawXAccel = (current.vxMetersPerSecond - lastFieldSpeeds.vxMetersPerSecond) / dt;
    double rawYAccel = (current.vyMetersPerSecond - lastFieldSpeeds.vyMetersPerSecond) / dt;
    double rawOmegaAccel = (current.omegaRadiansPerSecond - lastFieldSpeeds.omegaRadiansPerSecond) / dt;

    double filteredXAccel = xAccelFilter.calculate(rawXAccel);
    double filteredYAccel = yAccelFilter.calculate(rawYAccel);

    lastFieldSpeeds = current;
    lastTimestamp = now;

    return new ChassisSpeeds(filteredXAccel, filteredYAccel, rawOmegaAccel);
  }

  private void updateNetworktables() {
    if (swerveModuleStates != null) {
      // Create a sanitized copy replacing null entries (or null angles) with a safe
      // default
      SwerveModuleState[] safeStates = new SwerveModuleState[swerveModuleStates.length];
      for (int i = 0; i < swerveModuleStates.length; i++) {
        if (swerveModuleStates[i] == null) {
          safeStates[i] = new SwerveModuleState(0.0, new Rotation2d());
        } else if (swerveModuleStates[i].angle == null) {
          safeStates[i] = new SwerveModuleState(swerveModuleStates[i].speedMetersPerSecond, new Rotation2d());
        } else {
          safeStates[i] = swerveModuleStates[i];
        }
      }
      Logger.recordOutput("Swerve States", safeStates);

      ArrayList<Double> desiredStates = new ArrayList<>(8);

      for (int i = 0; i < swerveModuleStates.length; i++) {
        // Angle, Velocity / Module
        if (swerveModuleStates[i] != null) {
          desiredStates.add(swerveModuleStates[i].angle != null ? swerveModuleStates[i].angle.getDegrees() : 0);
          desiredStates.add(-swerveModuleStates[i].speedMetersPerSecond);
        } else {
          desiredStates.add(0.0);
          desiredStates.add(0.0);
        }
      }

      NetworkTables.desiredSwerveStates_da
          .setDoubleArray(desiredStates.stream().mapToDouble(Double::doubleValue).toArray());

      ArrayList<Double> measuredStates = new ArrayList<>(8);

      for (int i = 0; i < modules.length; i++) {
        // Angle, Velocity / Module
        measuredStates.add(modules[i].getTurnEncoder().getAbsolutePosition());
        measuredStates.add(-modules[i].getVelocity());
      }

      NetworkTables.measuredSwerveStates_da
          .setDoubleArray(measuredStates.stream().mapToDouble(Double::doubleValue).toArray());
    }

    NetworkTables.botRotDeg_d.setDouble(odometry.getGyroRotation().getDegrees());
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean locked) {
    if (states.length == 4) {
      for (int i = 0; i < 4; i++) {
        if (locked) {
          states[i].angle = Rotation2d.fromDegrees(Constants.Bot.lockedAngles[i]);
          states[i].speedMetersPerSecond = 0;
          swerveModuleStates[i] = states[i];
        }

        modules[i].setStates(states[i]);
      }
    } else {
      System.err.println("To many or too few swerve module states. NOT SETTING!");
    }
  }

  public Pose2d getPose() {
    return odometry.getPose();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPose(pose);
  }
}
