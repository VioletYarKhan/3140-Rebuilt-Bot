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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.libs.FuelSim;
import frc.robot.libs.NetworkTables;
import frc.robot.libs.Vector2;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TestRunner;
import frc.robot.subsystems.TestRunner.TestType;
import frc.robot.subsystems.odometry.Odometry;

public class TurretMain extends SubsystemBase {
  /////////////////////////////////////////////////////////////////////
  /// TUNING VOLTAGE OVERRIDE ///
  /////////////////////////////////////////////////////////////////////
  public static boolean flywheelRPMOverride = false;
  public static boolean hoodAngleOverride = false;

  private Pose3d turretPose = Constants.SIM.turretMechOffset;

  private Pose3d hoodPose = Constants.SIM.hoodMechOffset;

  SparkMax turretRotationMotor = new SparkMax(Constants.MotorIDs.turretRotation, SparkMax.MotorType.kBrushless); // Neo
  SparkFlex flywheelMotor = new SparkFlex(Constants.MotorIDs.flywheelMotor, SparkMax.MotorType.kBrushless); // Vortex
  SparkMax hoodMotor = new SparkMax(Constants.MotorIDs.hoodMotor, SparkMax.MotorType.kBrushless); // MiniNeo

  public SparkMaxSim turretRotationMotorSim;
  public SparkFlexSim flywheelMotorSim;
  public SparkMaxSim hoodMotorSim;
  // ROBOT WIN = TRUE
  public DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(Constants.SensorIDs.hoodEncoder,
      360 / Constants.Bot.hoodGearRatio, -21 + 77.5);
  public DutyCycleEncoderSim hoodEncoderSim = new DutyCycleEncoderSim(hoodEncoder);
  public Encoder turretEncoder = new Encoder(Constants.SensorIDs.turretEncoderA, Constants.SensorIDs.turretEncoderB);
  public EncoderSim turretEncoderSim = new EncoderSim(turretEncoder);

  private double getAngleDifference(double angleA, double angleB) {
    double diff = angleA - angleB;
    diff = ((diff + 180) % 360 + 360) % 360 - 180; // Normalize to [-180, 180]
    return diff;
  }

  public InterpolatingDoubleTreeMap hoodAngleToProjectileAngle = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap projectileAngleToHoodAngle = new InterpolatingDoubleTreeMap();

  public InterpolatingDoubleTreeMap flywheelSpeedToProjectileSpeed = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap projectileSpeedToFlywheelSpeed = new InterpolatingDoubleTreeMap();

  public double flywheelAdjustmantConstantM = 0.5;
  public double flywheelAdjustmantConstantB = 1700;

  double lastUpdateTimestamp = 0;

  public static final double stowRange = .2; // meters

  @AutoLogOutput
  double hoodSetpoint = 0; // degrees
  @AutoLogOutput
  double flywheelSetpoint = 0; // RPM
  @AutoLogOutput
  public double turretSetpoint = 0; // degrees
  @AutoLogOutput
  boolean shouldShoot = false;

  boolean shouldShootMode = false;

  public PIDController hoodPID = new PIDController(0, 0, 0);
  public LoggedPIDInputs hoodPIDInputs = new LoggedPIDInputs("HoodPID",
      Constants.PID.Turret.hoodP,
      Constants.PID.Turret.hoodI,
      Constants.PID.Turret.hoodD);

  public Constraints turretConstraints = new TrapezoidProfile.Constraints(0, 0);
  public ProfiledPIDController turretPID = new ProfiledPIDController(0, 0, 0, turretConstraints);
  public LoggedPIDInputs turretPIDInputs = new LoggedPIDInputs("RotationPID",
      Constants.PID.Turret.turretP,
      Constants.PID.Turret.turretI,
      Constants.PID.Turret.turretD);

  public SimpleMotorFeedforward turretFeedforward;
  public LoggedPIDInputs turretFeedforwardInputs = new LoggedPIDInputs("TurretFeedfoward",
      Constants.FeedFoward.Turret.turretS,
      Constants.FeedFoward.Turret.turretV,
      Constants.FeedFoward.Turret.turretA);

  public SimpleMotorFeedforward flywheelFeedforward;
  public LoggedPIDInputs flywheelFeedforwardInputs = new LoggedPIDInputs("FlywheelFeedforward",
      Constants.FeedFoward.Turret.flywheelS,
      Constants.FeedFoward.Turret.flywheelV,
      Constants.FeedFoward.Turret.flywheelA);
  private boolean spinup = false;
  private double flywheelSpeedTolerance = 500; // RPM

  private double flywheelFeedbackP = Constants.PID.Turret.flywheelP;

  public enum AimOpt {
    AUTO,
    MANUAL
  }

  public HashMap<AimOpt, AimType> aimTypes = new HashMap<AimOpt, AimType>();

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

    hoodAngleToProjectileAngle.put(5.0, 90.0);
    hoodAngleToProjectileAngle.put(15.0, 80.0);
    hoodAngleToProjectileAngle.put(32.5, 56.15);
    hoodAngleToProjectileAngle.put(37.5, 50.00);

    hoodAngleToProjectileAngle.put(90.0, 4.0);
    projectileAngleToHoodAngle.put(80.0, 15.0);
    projectileAngleToHoodAngle.put(56.15, 32.5);
    projectileAngleToHoodAngle.put(50.00, 37.5);

    projectileSpeedToFlywheelSpeed.put(0.0, 1000.0);
    projectileSpeedToFlywheelSpeed.put(5.18226, 3000.0);
    projectileSpeedToFlywheelSpeed.put(6.4485563, 3500.0);
    projectileSpeedToFlywheelSpeed.put(7.7033197, 4000.0);
    projectileSpeedToFlywheelSpeed.put(8.6498802, 4500.0);
    projectileSpeedToFlywheelSpeed.put(9.7442698, 5000.0);
    projectileSpeedToFlywheelSpeed.put(10.557472, 5500.0);
    projectileSpeedToFlywheelSpeed.put(11.04, 6000.0);
    projectileSpeedToFlywheelSpeed.put(11.5, 7000.0);

    flywheelSpeedToProjectileSpeed.put(1000.0, 0.0);
    flywheelSpeedToProjectileSpeed.put(3000.0, 5.18226);
    flywheelSpeedToProjectileSpeed.put(3500.0, 6.4485563);
    flywheelSpeedToProjectileSpeed.put(4000.0, 7.7033197);
    flywheelSpeedToProjectileSpeed.put(4500.0, 8.6498802);
    flywheelSpeedToProjectileSpeed.put(5000.0, 9.7442698);
    flywheelSpeedToProjectileSpeed.put(5500.0, 10.557472);
    flywheelSpeedToProjectileSpeed.put(6000.0, 11.04);
    flywheelSpeedToProjectileSpeed.put(7000.0, 11.5);

    NetworkTables.flywheelRPMOverride_d.setDouble(5000);
    NetworkTables.flywheelAdjustmantConstantM_d.setDouble(flywheelAdjustmantConstantM);
    NetworkTables.flywheelAdjustmantConstantB_d.setDouble(flywheelAdjustmantConstantB);
    NetworkTables.hoodAngle_d.setDouble(0);
    NetworkTables.flywheelPVoltage_d.setDouble(0);

    hoodPID.setPID(hoodPIDInputs.getP(), hoodPIDInputs.getI(), hoodPIDInputs.getD());
    hoodPID.enableContinuousInput(0, 360);
    turretPID.setPID(turretPIDInputs.getP(), turretPIDInputs.getI(), turretPIDInputs.getD());
    flywheelFeedforward = new SimpleMotorFeedforward(
        flywheelFeedforwardInputs.getP(),
        flywheelFeedforwardInputs.getI(),
        flywheelFeedforwardInputs.getD());

    turretFeedforward = new SimpleMotorFeedforward(
        turretFeedforwardInputs.getP(),
        turretFeedforwardInputs.getI(),
        turretFeedforwardInputs.getD());

    turretPID.setIntegratorRange(-0.1, 0.1);
    hoodPID.setIntegratorRange(-0.1, 0.1);
    SparkMaxConfig turretConfig = new SparkMaxConfig();
    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    turretEncoder.setDistancePerPulse(1.04 / (Constants.Bot.turretGearRatio * Constants.Bot.turretGearRatio));
    turretConfig.idleMode(IdleMode.kCoast).inverted(true).smartCurrentLimit(Constants.CurrentLimits.Turret.turretLimit);
    hoodConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(Constants.CurrentLimits.Turret.hoodLimit);
    flywheelConfig.idleMode(IdleMode.kCoast).inverted(true)
        .smartCurrentLimit(Constants.CurrentLimits.Turret.flywheelLimit);
    flywheelConfig.closedLoop.pid(Constants.PID.Turret.flywheelP, 
                                  Constants.PID.Turret.flywheelI, 
                                  Constants.PID.Turret.flywheelD)
      .feedForward.kS(Constants.FeedFoward.Turret.flywheelS)
                  .kV(Constants.FeedFoward.Turret.flywheelV);

    turretRotationMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    aimTypes.put(AimOpt.AUTO, new AutoAim(hoodAngleToProjectileAngle.get(Constants.Limits.Turret.maxPitch),
        hoodAngleToProjectileAngle.get(Constants.Limits.Turret.minPitch)));
    aimTypes.put(AimOpt.MANUAL, new ManualAim());
    System.out.println("Min: " + hoodAngleToProjectileAngle.get(Constants.Limits.Turret.minPitch) + " Max: "
        + hoodAngleToProjectileAngle.get(Constants.Limits.Turret.maxPitch));

    turretSetpoint = getTurretEncoderAngle();
    clampTurretSetpoint();

    hoodSetpoint = 0;
    flywheelSetpoint = 0;

    aimTypes.get(currentMode).activate(turretSetpoint, hoodAngleToProjectileAngle.get(hoodSetpoint),
        FlywheelRPMToSpeed(flywheelSetpoint));

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
      aimTypes.get(currentMode).activate(turretSetpoint, hoodAngleToProjectileAngle.get(hoodSetpoint),
          FlywheelRPMToSpeed(flywheelSetpoint));
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
  }

  public boolean flywheelAtSpeed() {
    return Math.abs(flywheelSetpoint - flywheelMotor.getEncoder().getVelocity()) <= flywheelSpeedTolerance;
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
    return hoodEncoder.get();
  }

  private double getProjectileAngle() {
    return hoodAngleToProjectileAngle.get(hoodEncoder.get());
  }

  public void setHoodAngle(double angle) {
    hoodSetpoint = angle;
  }

  public void setProjectileAngle(double angle) {
    hoodSetpoint = projectileAngleToHoodAngle.get(angle);
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

    return shouldStow || Controller.getInstance().secondaryController.getBButton();
  }

  @SuppressWarnings("unused")
  @Override
  public void periodic() {

    hoodPID.setPID(hoodPIDInputs.getP(), hoodPIDInputs.getI(), hoodPIDInputs.getD());
    turretPID.setPID(turretPIDInputs.getP(), turretPIDInputs.getI(), turretPIDInputs.getD());
    flywheelFeedforward.setKs(flywheelFeedforwardInputs.getP());
    flywheelFeedforward.setKv(flywheelFeedforwardInputs.getI());
    flywheelFeedforward.setKa(flywheelFeedforwardInputs.getD());

    turretFeedforward.setKs(turretFeedforwardInputs.getP());
    turretFeedforward.setKv(turretFeedforwardInputs.getI());
    turretFeedforward.setKa(turretFeedforwardInputs.getD());

    flywheelAdjustmantConstantB = NetworkTables.flywheelAdjustmantConstantB_d.getDouble(0);
    flywheelAdjustmantConstantM = NetworkTables.flywheelAdjustmantConstantM_d.getDouble(1);
    flywheelFeedbackP = NetworkTables.flywheelPVoltage_d.getDouble(0);

    double turretEnc = getTurretEncoderAngle();
    hoodPIDInputs.update(hoodSetpoint, getHoodEncoderAngle());
    turretPIDInputs.update(turretSetpoint, turretEnc);
    flywheelFeedforwardInputs.update(flywheelSetpoint, flywheelMotor.getEncoder().getVelocity());
    flywheelFeedforwardInputs.update(flywheelSetpoint, flywheelMotor.getEncoder().getVelocity());

    if (!TestRunner.getInstance().isRunning(TestType.TURRET)) {
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
            getProjectileAngle(),
            FlywheelRPMToSpeed(flywheelMotor.getEncoder().getVelocity()),
            getTurretEncoderAngle());

        flywheelSetpoint = SpeedToFlywheelRPM(type.flywheelSpeed); // convert from m/s to RPM
        hoodSetpoint = projectileAngleToHoodAngle.get(type.hoodAngle);
        if (currentMode == AimOpt.MANUAL) {
          hoodSetpoint = type.hoodAngle;
        }
        hoodSetpoint = Math.max(Constants.Limits.Turret.minPitch,
            Math.min(Constants.Limits.Turret.maxPitch, hoodSetpoint));
        turretSetpoint = type.rotationAngle;
        boolean hadToClamp = clampTurretSetpoint();
        shouldShootMode = type.shouldShoot;// && !hadToClamp;
      }
    }

    boolean stow = shouldStow() || !spinup;
    shouldShoot = shouldShootMode && !stow;

    hoodPID.setSetpoint(hoodAngleOverride ? NetworkTables.hoodAngle_d.getDouble(0) : stow ? 352 : hoodSetpoint);
    hoodMotor.set(hoodPID.calculate(getHoodEncoderAngle()));

    double encoderValue = getTurretEncoderAngle();
    if(AimOpt.AUTO == currentMode) turretSetpoint = spinup ? turretSetpoint : encoderValue;
    double output = turretPID.calculate(encoderValue,
        Math.min(Constants.Limits.Turret.maxYaw, Math.max(turretSetpoint, Constants.Limits.Turret.minYaw)));
    output += turretFeedforward.calculate(turretPID.getSetpoint().velocity);
    turretRotationMotor.set(output);

    if (flywheelRPMOverride) {
      flywheelSetpoint = NetworkTables.flywheelRPMOverride_d.getDouble(0);
    }

    if (spinup) {
      flywheelMotor.getClosedLoopController().setSetpoint(Math.max(0, flywheelSetpoint), SparkMax.ControlType.kVelocity);
    } else {
      flywheelMotor.getClosedLoopController().setSetpoint(0, SparkMax.ControlType.kVoltage);
    }

    hoodPose = new Pose3d(
        Constants.SIM.hoodMechOffset.getX(),
        Constants.SIM.hoodMechOffset.getY(),
        Constants.SIM.hoodMechOffset.getZ(),
        new Rotation3d(0, Math.toRadians(getHoodEncoderAngle()),
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
    return flywheelSpeedToProjectileSpeed.get(RPM) * flywheelAdjustmantConstantM + flywheelAdjustmantConstantB;
  }

  public double SpeedToFlywheelRPM(double speed) {
    return (projectileSpeedToFlywheelSpeed.get(speed) - flywheelAdjustmantConstantB) / flywheelAdjustmantConstantM;
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

  double lastShotTime = 0;
  double shotDelay = 0.2;

  public void shootSimFuel() {
    if (Timer.getFPGATimestamp() - lastShotTime < shotDelay)
      return;
    lastShotTime = Timer.getFPGATimestamp();

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
        Math.toRadians(getProjectileAngle()),
        Math.toRadians(
            getTurretEncoderAngle() + robotFieldPose.getRotation().getDegrees()));

    Pose3d shooterPose = new Pose3d(fieldX, fieldY, fieldZ, shooterRot);

    // Calculate robot's velocity direction (field-relative)
    double robotVelX = Odometry.getInstance().getBotVelocity(true).X; // implement or replace with your method
    double robotVelY = Odometry.getInstance().getBotVelocity(true).Y; // implement or replace with your method
    double robotVelZ = 0; // usually 0 unless you have a swerve module that can jump :)

    // Calculate projectile speed (magnitude)
    double projectileSpeed = flywheelSpeedToProjectileSpeed.get(flywheelMotor.getEncoder().getVelocity()) * 0.5 + 3.5;

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

    if (Intake.getInstance().getBalls() != 0){
      FuelSim.getInstance().spawnFuel(
        shooterPose.getTranslation(), new Translation3d(vx, vy, vz));
      Intake.getInstance().removeBall();
    }
    

    /*
    double projectileSpeed = flywheelSpeedToProjectileSpeed.get(flywheelMotor.getEncoder().getVelocity());
    double pitch = Math.toRadians(getProjectileAngle());
    double turretYaw = Math.toRadians(getTurretEncoderAngle());

    RobotContainer.fuelSim.launchFuel(
        projectileSpeed,
        pitch,
        turretYaw,
        Units.inchesToMeters(14));
        */
  }
}
