// Copyright (c) FIRST and other WPILib contributors.
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import java.util.ArrayList;
import java.util.HashMap;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.libs.NetworkTables;
import frc.robot.libs.Vector2;
import frc.robot.subsystems.TestRunner;
import frc.robot.subsystems.TestRunner.TestType;
import frc.robot.subsystems.odometry.Odometry;

public class TurretMain extends SubsystemBase {
  /////////////////////////////////////////////////////////////////////
  /// TUNING VOLTAGE OVERRIDE ///
  /////////////////////////////////////////////////////////////////////
  public static final boolean flywheelVoltOverride = true;
  private final boolean hoodAngleOverride = true;

  private Pose3d turretPose = Constants.SIM.turretMechOffset;

  private Pose3d hoodPose = Constants.SIM.hoodMechOffset;

  SparkMax turretRotationMotor = new SparkMax(Constants.MotorIDs.turretRotation, SparkMax.MotorType.kBrushless); // Neo
  SparkFlex flywheelMotor = new SparkFlex(Constants.MotorIDs.flywheelMotor, SparkMax.MotorType.kBrushless); // Vortex
  SparkMax hoodMotor = new SparkMax(Constants.MotorIDs.hoodMotor, SparkMax.MotorType.kBrushless); // MiniNeo

  public SparkMaxSim turretRotationMotorSim;
  public SparkFlexSim flywheelMotorSim;
  public SparkMaxSim hoodMotorSim;
  // ROBOT WIN = TRUE
  public DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(Constants.SensorIDs.hoodEncoder, 1, 0);
  public DutyCycleEncoderSim hoodEncoderSim = new DutyCycleEncoderSim(hoodEncoder);
  public Encoder turretEncoder = new Encoder(Constants.SensorIDs.turretEncoderA, Constants.SensorIDs.turretEncoderB);

  double lastUpdateTimestamp = 0;

  public static final double stowRange = 0.6; // meters

  @AutoLogOutput
  double hoodSetpoint = 0; // degrees
  @AutoLogOutput
  double flywheelSetpoint = 0; // RPM
  @AutoLogOutput
  double turretSetpoint = 0; // degrees
  @AutoLogOutput
  boolean shouldShoot = false;

  boolean shouldShootMode = false;

  public PIDController hoodPID = new PIDController(0, 0, 0);
  public LoggedPIDInputs hoodPIDInputs = new LoggedPIDInputs("HoodPID",
      Constants.PID.Turret.hoodP,
      Constants.PID.Turret.hoodI,
      Constants.PID.Turret.hoodD);

  public PIDController rotationProfiledPID = new PIDController(0, 0, 0);
  public LoggedPIDInputs rotationPIDInputs = new LoggedPIDInputs("RotationPID",
      Constants.PID.Turret.rotationP,
      Constants.PID.Turret.rotationI,
      Constants.PID.Turret.rotationD);

  public SimpleMotorFeedforward flywheelFeedforward;
  public LoggedPIDInputs flywheelFeedfowardInputs = new LoggedPIDInputs("FlywheelFeedforward",
      Constants.FeedFoward.Turret.flywheelS,
      Constants.FeedFoward.Turret.flywheelV,
      Constants.FeedFoward.Turret.flywheelA);
  private boolean spinup = false;

  private static double RPMSpeedConversion = 0.0762 * 2 * Math.PI / 60; // convert from m/s to RPM

  public enum AimOpt {
    AUTO,
    MANUAL
  }

  private HashMap<AimOpt, AimType> aimTypes = new HashMap<AimOpt, AimType>();

  private AimOpt currentMode = AimOpt.AUTO;

  private static TurretMain m_instance = null;

  public static TurretMain getInstance() {
    if (m_instance == null) {
      m_instance = new TurretMain();
    }
    return m_instance;
  }

  private ArrayList<Fuel> gamePieces = new ArrayList<Fuel>();
  private ArrayList<Pose3d> publishedGamePieces = new ArrayList<Pose3d>();

  public static class LoggedPIDInputs {
    private final LoggedNetworkNumber kP;
    private final LoggedNetworkNumber kI;
    private final LoggedNetworkNumber kD;
    private final LoggedNetworkNumber error;
    private final LoggedNetworkNumber setpoint;
    private final LoggedNetworkNumber measurement;

    public LoggedPIDInputs(String name, double defaultP, double defaultI, double defaultD) {
      kP = new LoggedNetworkNumber("PID/" + name + "/kP", defaultP);
      kI = new LoggedNetworkNumber("PID/" + name + "/kI", defaultI);
      kD = new LoggedNetworkNumber("PID/" + name + "/kD", defaultD);
      error = new LoggedNetworkNumber("PID/" + name + "/error", defaultD);
      setpoint = new LoggedNetworkNumber("PID/" + name + "/setpoint", 0);
      measurement = new LoggedNetworkNumber("PID/" + name + "/measurement", 0);
    }

    public double getP() {
      return kP.get();
    }

    public double getI() {
      return kI.get();
    }

    public double getD() {
      return kD.get();
    }

    public void update(double sp, double m) {
      setpoint.set(sp);
      measurement.set(m);
      error.set(Math.abs(sp - m));
    }
  }

  private class Fuel {
    private static final double GRAVITY = -9.81; // m/s^2, downward

    public Pose3d position;
    public double vx, vy, vz; // velocity components

    public Fuel(Pose3d pos, double velocity) {
      this.position = pos;

      // Calculate launch direction from pose rotation (assume launch along +X axis)
      Rotation3d rot = pos.getRotation();
      double pitch = rot.getY(); // radians
      double yaw = rot.getZ(); // radians

      double dx = Math.cos(pitch) * Math.cos(yaw);
      double dy = Math.cos(pitch) * Math.sin(yaw);
      double dz = Math.sin(pitch);

      // Set velocity vector in facing direction
      vx = velocity * dx;
      vy = velocity * dy;
      vz = velocity * dz;
    }

    // Step simulation by dt seconds
    public void step(double dt) {
      // Update velocity with gravity (only z)
      vz += GRAVITY * dt;

      // Update position
      double newX = position.getX() + vx * dt;
      double newY = position.getY() + vy * dt;
      double newZ = position.getZ() + vz * dt;

      // Update pose with new position, keep rotation
      position = new Pose3d(newX, newY, newZ, position.getRotation());

      // Optionally, update rotation to match velocity direction
      // If you want the pose's rotation to follow the velocity vector:
      double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
      if (speed > 1e-6) {
        double pitch = Math.asin(vz / speed);
        double yaw = Math.atan2(vy, vx);
        position = new Pose3d(newX, newY, newZ, new Rotation3d(0, pitch, yaw));
      }
    }
  }

  /** Creates a new Turret. */
  public TurretMain() {
    if (flywheelVoltOverride) {
      NetworkTables.flywheelTuningVoltage_d.setDouble(5.000);
    }
    if (hoodAngleOverride) {
      NetworkTables.hoodAngle_d.setDouble(40);
    }

      NetworkTables.flywheelRPMConversionConstant_d.setDouble(0.1);
    hoodPID.setPID(hoodPIDInputs.getP(), hoodPIDInputs.getI(), hoodPIDInputs.getD());
    rotationProfiledPID.setPID(rotationPIDInputs.getP(), rotationPIDInputs.getI(), rotationPIDInputs.getD());
    flywheelFeedforward = new SimpleMotorFeedforward(
        flywheelFeedfowardInputs.getP(),
        flywheelFeedfowardInputs.getI(),
        flywheelFeedfowardInputs.getD());

    // hoodPID.enableContinuousInput(0, 360);

    SparkMaxConfig turretConfig = new SparkMaxConfig();
    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    turretEncoder.setDistancePerPulse(1/(Constants.Bot.turretGearRatio*Constants.Bot.turretGearRatio));
    turretConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(Constants.CurrentLimits.Turret.turretLimit);
    hoodConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(Constants.CurrentLimits.Turret.hoodLimit);
    flywheelConfig.idleMode(IdleMode.kCoast).inverted(true).smartCurrentLimit(Constants.CurrentLimits.Turret.flywheelLimit);

    turretConfig.smartCurrentLimit(Constants.CurrentLimits.Turret.turretLimit);

    turretRotationMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    aimTypes.put(AimOpt.AUTO, new AutoAim());
    aimTypes.put(AimOpt.MANUAL, new ManualAim());

    turretSetpoint = getTurretEncoderAngle();
    clampTurretSetpoint();

    hoodSetpoint = 90;
    flywheelSetpoint = 0;

    aimTypes.get(currentMode).activate(turretSetpoint, hoodSetpoint, FlywheelRPMToSpeed(flywheelSetpoint));

    if (RobotBase.isSimulation()) {
      turretRotationMotorSim = new SparkMaxSim(turretRotationMotor, DCMotor.getNEO(1));
      flywheelMotorSim = new SparkFlexSim(flywheelMotor, DCMotor.getNEO(1));
      hoodMotorSim = new SparkMaxSim(hoodMotor, DCMotor.getNEO(1));
    }
  }

  public void setAimMode(AimOpt mode) {
    if (mode != currentMode) {
      aimTypes.get(currentMode).deactivate();
      currentMode = mode;
      aimTypes.get(currentMode).activate(turretSetpoint, hoodSetpoint, FlywheelRPMToSpeed(flywheelSetpoint));
    }
  }

  public void setFlywheelActive(boolean active) {
    spinup = active;
  }

  public boolean getFlywheelActive() {
    return spinup;
  }

  public boolean shouldShoot() {
    return shouldShoot;
    // && Math.abs(flywheelSetpoint - flywheelMotor.getEncoder().getVelocity()) <=
    // flywheelSpeedTolerance;
  }

  private double getTurretEncoderAngle() {
    double encoderValue = -turretEncoder.getDistance();
    while (encoderValue > 180) {
      encoderValue -= 360;
    }
    while (encoderValue <= -180) {
      encoderValue += 360;
    }
    return encoderValue;
  }

  private double getHoodEncoderAngle() {
    double encoderValue = hoodEncoder.get()*360/Constants.Bot.hoodGearRatio - Constants.Bot.hoodZeroOffset;
    return 90 - encoderValue;
  }

  public void setHoodAngle(double angle) {
    hoodSetpoint = angle;
  }

  public void setRotationAngle(double angle) {
    turretSetpoint = angle;
    clampTurretSetpoint();
  }

  private boolean shouldStow() {
    Vector2 botPosition = Odometry.getInstance().getPosition();
    Vector2 turretDirection = Constants.PathplannerConstants.botTurretOffset.rotate(Odometry.getInstance().getAngle());
    Vector2 turretPosition = botPosition.add(turretDirection);

    boolean shouldStow = Math.abs(turretPosition.X - Constants.PathplannerConstants.blueTrenchX) < stowRange ||
        Math.abs(turretPosition.X - Constants.PathplannerConstants.redTrenchX) < stowRange;

    return shouldStow;
  }

  @SuppressWarnings("unused")
  @Override
  public void periodic() {

    hoodPID.setPID(hoodPIDInputs.getP(), hoodPIDInputs.getI(), hoodPIDInputs.getD());
    rotationProfiledPID.setPID(rotationPIDInputs.getP(), rotationPIDInputs.getI(), rotationPIDInputs.getD());
    flywheelFeedforward.setKs(flywheelFeedfowardInputs.getP());
    flywheelFeedforward.setKv(flywheelFeedfowardInputs.getI());
    flywheelFeedforward.setKa(flywheelFeedfowardInputs.getD());

    double turretEnc = getTurretEncoderAngle();
    hoodPIDInputs.update(hoodSetpoint, getHoodEncoderAngle());
    rotationPIDInputs.update(turretSetpoint, turretEnc);
    flywheelFeedfowardInputs.update(flywheelSetpoint, flywheelMotor.getEncoder().getVelocity());

    rotationProfiledPID.setSetpoint(turretSetpoint);

    if (!TestRunner.getInstance().isRunning(TestType.TURRET)) {
      // TODO: Default Hood Angle DN, Manual Mode, Limiting Angle
      if (lastUpdateTimestamp == 0) {
        lastUpdateTimestamp = Timer.getFPGATimestamp();
        // first update, setup
        shouldShootMode = false;
      } else {
        double t = Timer.getFPGATimestamp();
        double deltaTime = t - lastUpdateTimestamp;
        lastUpdateTimestamp = t;

        AimType type = aimTypes.get(currentMode);

        type.periodic(
            deltaTime,
            getHoodEncoderAngle(),
            FlywheelRPMToSpeed(flywheelMotor.getEncoder().getVelocity()),
            getTurretEncoderAngle());

        flywheelSetpoint = FlywheelSpeedToRPM(type.flywheelSpeed); // convert from m/s to RPM
        hoodSetpoint = type.hoodAngle;
        turretSetpoint = type.rotationAngle;
        boolean hadToClamp = clampTurretSetpoint();
        shouldShootMode = type.shouldShoot && hadToClamp;
      }
    }

    boolean stow = shouldStow();
    shouldShoot = shouldShootMode && !stow;

    hoodPID.setSetpoint(hoodAngleOverride ? NetworkTables.hoodAngle_d.getDouble(70) : stow ? 70: hoodSetpoint);
    rotationProfiledPID.setSetpoint(turretSetpoint);

    hoodMotor.set(hoodPID.calculate(getHoodEncoderAngle()));

    double encoderValue = getTurretEncoderAngle();
    // System.out.println("Enc: "+encoderValue+"\tSetp: "+turretSetpoint);
    turretRotationMotor.set(rotationProfiledPID.calculate(encoderValue));
    if(flywheelVoltOverride) {
        flywheelSetpoint = FlywheelSpeedToRPM(NetworkTables.flywheelTuningVoltage_d.getDouble(0)); 
    }
    if (spinup) {
      flywheelMotor.set(flywheelFeedforward.calculate(flywheelSetpoint) / 12);
    } else {
      flywheelMotor.setVoltage(0);
    }

    hoodPose = new Pose3d(
        Constants.SIM.hoodMechOffset.getX(),
        Constants.SIM.hoodMechOffset.getY(),
        Constants.SIM.hoodMechOffset.getZ(),
        new Rotation3d(0, Math.toRadians(90 - getHoodEncoderAngle()),
            Math.toRadians(encoderValue)));
    turretPose = new Pose3d(
        Constants.SIM.turretMechOffset.getX(),
        Constants.SIM.turretMechOffset.getY(),
        Constants.SIM.turretMechOffset.getZ(),
        new Rotation3d(0, 0, Math.toRadians(encoderValue)));

    Robot.mecanismPoses[1] = turretPose;
    Robot.mecanismPoses[2] = hoodPose;
  }

  private boolean clampTurretSetpoint() {
    while (turretSetpoint <= -180) {
      turretSetpoint += 360;
    }
    while (turretSetpoint > 180) {
      turretSetpoint -= 360;
    }

    if (turretSetpoint < Constants.Limits.Turret.minYaw) {
      turretSetpoint = Constants.Limits.Turret.minYaw;
      return false;
    } else if (turretSetpoint > Constants.Limits.Turret.maxYaw) {
      turretSetpoint = Constants.Limits.Turret.maxYaw;
      return false;
    }

    return true;
  }

  public double getLookDirection() {
    return aimTypes.get(currentMode).getLookDirection() + 180;
  }

  public double FlywheelRPMToSpeed(double RPM) {
    return RPM * RPMSpeedConversion * NetworkTables.flywheelRPMConversionConstant_d.getDouble(1);
  }

  public double FlywheelSpeedToRPM(double speed) {
    return speed / (RPMSpeedConversion * NetworkTables.flywheelRPMConversionConstant_d.getDouble(1));
  }

  public void simFuel(double dt) {

    for (int i = 0; i < gamePieces.size(); i++) {
      publishedGamePieces.set(i, gamePieces.get(i).position);
      if (gamePieces.get(i).position.getZ() > 0) {
        gamePieces.get(i).step(dt);
      } else {
        publishedGamePieces.remove(i);
        gamePieces.remove(i);
      }
    }

    Logger.recordOutput("GamePieces", publishedGamePieces.toArray(new Pose3d[0]));
  }

  public void shootSimFuel() {
    if (!shouldShoot())
      return;

    // Get robot's field pose (x, y, rotation)
    Pose2d robotFieldPose = Odometry.getInstance().getRealSimPose();
    Pose3d turretOffset = Constants.SIM.hoodMechOffset;

    // Rotate turret offset by robot heading (about Z axis)
    double headingRad = robotFieldPose.getRotation().getRadians();
    double cosH = Math.cos(headingRad);
    double sinH = Math.sin(headingRad);
    double offsetX = turretOffset.getX() * cosH - turretOffset.getY() * sinH;
    double offsetY = turretOffset.getX() * sinH + turretOffset.getY() * cosH;
    double offsetZ = turretOffset.getZ();

    // Add to robot's field position
    double fieldX = robotFieldPose.getX() + offsetX;
    double fieldY = robotFieldPose.getY() + offsetY;
    double fieldZ = offsetZ;

    // Build the shooter pose at the correct field position, with turret/hood
    // rotation
    Rotation3d shooterRot = new Rotation3d(
        0,
        Math.toRadians(getHoodEncoderAngle()),
        Math.toRadians(
            getTurretEncoderAngle() + robotFieldPose.getRotation().getDegrees()));

    Pose3d shooterPose = new Pose3d(fieldX, fieldY, fieldZ, shooterRot);

    // Calculate robot's velocity direction (field-relative)
    double robotVelX = Odometry.getInstance().getBotVelocity(true).X; // implement or replace with your method
    double robotVelY = Odometry.getInstance().getBotVelocity(true).Y; // implement or replace with your method
    double robotVelZ = 0; // usually 0 unless you have a swerve module that can jump :)

    // Calculate projectile speed (magnitude)
    double projectileSpeed = FlywheelRPMToSpeed(flywheelMotor.getEncoder().getVelocity());

    // Calculate launch direction from shooter pose
    Rotation3d rot = shooterPose.getRotation();
    double pitch = rot.getY(); // radians
    double yaw = rot.getZ(); // radians
    double dx = Math.cos(pitch) * Math.cos(yaw);
    double dy = Math.cos(pitch) * Math.sin(yaw);
    double dz = Math.sin(pitch);

    // Add tangential velocity
    Vector2 turretPosition = new Vector2(turretPose.toPose2d().getX(), turretPose.toPose2d().getY())
        .rotate(Odometry.getInstance().getRotation().getRadians() + (Math.PI / 2))
        .mult(Odometry.getInstance().getAngularVelocity());

    // Add robot velocity to projectile velocity
    double vx = projectileSpeed * dx + robotVelX + turretPosition.X;
    double vy = projectileSpeed * dy + robotVelY + turretPosition.Y;
    double vz = projectileSpeed * dz + robotVelZ;

    Fuel fuel = new Fuel(shooterPose, 0);
    fuel.vx = vx;
    fuel.vy = vy;
    fuel.vz = vz;
    gamePieces.add(fuel);
    publishedGamePieces.add(shooterPose);
  }
}
