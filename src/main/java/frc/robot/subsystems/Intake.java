// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.libs.NetworkTables;
import frc.robot.subsystems.Turret.TurretMain;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private Pose3d armPose = Constants.SIM.intakeMechOffset;

  private SparkMax intakeArmMotorL = new SparkMax(Constants.MotorIDs.intakeArmMotor, SparkMax.MotorType.kBrushless);
  private SparkMax intakeArmMotorR = new SparkMax(Constants.MotorIDs.intakeArmMotorFollower,
      SparkMax.MotorType.kBrushless);
  private SparkMax intakeRollerMotor = new SparkMax(Constants.MotorIDs.intakeMotor, SparkMax.MotorType.kBrushless);
  public SparkMaxSim intakeArmMotorSim;

  private double rightEncoderOffset = -0.132 - 0.1895 - 0.114;

  public DutyCycleEncoder intakeEncoderL = new DutyCycleEncoder(Constants.SensorIDs.intakeEncoderL, 1,
      Constants.Limits.Intake.leftOffset);
  public DutyCycleEncoder intakeEncoderR = new DutyCycleEncoder(Constants.SensorIDs.intakeEncoderR, 1,
      rightEncoderOffset);
  public DutyCycleEncoderSim intakeEncoderSim = new DutyCycleEncoderSim(intakeEncoderL);

  private double gravityFeedFowardConstant = 0;
  private double separationConstant = Robot.isReal() ? 2 : 0.000;
  private double separationConstantThreshold = 4;
  private double intakeSetpoint = Constants.Limits.Intake.stowedPosition;

  private int simBalls = 8;

  private TurretMain.LoggedPIDInputs intakePIDInputs = new TurretMain.LoggedPIDInputs(
      "Intake",
      Constants.PID.Intake.intakeP,
      Constants.PID.Intake.intakeI,
      Constants.PID.Intake.intakeD);

  private PIDController intakePIDL = new PIDController(
      Constants.PID.Intake.intakeP,
      Constants.PID.Intake.intakeI,
      Constants.PID.Intake.intakeD);
  private PIDController intakePIDR = new PIDController(
      Constants.PID.Intake.intakeP,
      Constants.PID.Intake.intakeI,
      Constants.PID.Intake.intakeD);

  private static Intake m_instance = null;

  public static Intake getInstance() {
    if (m_instance == null) {
      m_instance = new Intake();
    }
    return m_instance;
  }

  /** Creates a new Intake. */
  public Intake() {
    SparkMaxConfig config = new SparkMaxConfig();

    intakePIDL.enableContinuousInput(0, 1);
    intakePIDR.enableContinuousInput(0, 1);

    intakePIDL.setTolerance(0.1);
    intakePIDR.setTolerance(0.1);

    intakeRollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(true);

    config.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(15);
    intakeArmMotorL.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(false);
    intakeArmMotorR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    NetworkTables.intakeGravityConstant.setDouble(gravityFeedFowardConstant);
    NetworkTables.intakeSeparationConstant.setDouble(separationConstant);
    NetworkTables.intakeRollerSpeed_d.setDouble(Constants.MotorSpeeds.Intake.intakeSpeed);

    if (RobotBase.isSimulation()) {
      intakeArmMotorSim = new SparkMaxSim(intakeArmMotorL, DCMotor.getNEO(1));
    }
  }

  public void deploy() {
    intakeSetpoint = Constants.Limits.Intake.deployedPosition;
  }

  public void feed() {
    intakeSetpoint = Constants.Limits.Intake.feedPosition;
  }

  public void stow() {
    intakeSetpoint = Constants.Limits.Intake.stowedPosition;
  }

  public boolean isStowed() {
    return intakeSetpoint == Constants.Limits.Intake.stowedPosition;
  }

  public boolean isFeeding() {
    return intakeSetpoint == Constants.Limits.Intake.feedPosition;
  }

  public boolean isDeployed() {
    return intakeSetpoint == Constants.Limits.Intake.deployedPosition;
  }

  public double getAngle() {
    return intakeEncoderL.get();
  }

  public double getRightAngle() {
    return 1 - intakeEncoderR.get();
  }

  /**
   * No need to pass in negative speed, method handles that (Ignores passed sign)
   * 
   * @param speed
   */
  public void intake(double speed) {
    intakeRollerMotor.set(Math.abs(speed));
  }

  /**
   * No need to pass in negative speed, method handles that (Ignores passed sign)
   * 
   * @param speed
   */
  public void outtake(double speed) {
    intakeRollerMotor.set(-Math.abs(speed));
  }

  public boolean isActive() {
    return intakeRollerMotor.get() > 0;
  }

  // Angle to the horizontal
  public double getLeftSideAngle() {
    return getAngle() * 360;
  }

  // Angle to the horizontal
  public double getRightSideAngle() {
    return getRightAngle() * 360;
  }

  private double getAngleDifference(double angleA, double angleB) {
    double diff = angleA - angleB;
    diff = ((diff + 180) % 360 + 360) % 360 - 180; // Normalize to [-180, 180]
    return diff;
  }

  @Override
  public void periodic() {
    intakePIDL.setP(intakePIDInputs.getP());
    intakePIDL.setI(intakePIDInputs.getI());
    intakePIDL.setD(intakePIDInputs.getD());

    intakePIDR.setP(intakePIDInputs.getP());
    intakePIDR.setI(intakePIDInputs.getI());
    intakePIDR.setD(intakePIDInputs.getD());

    intakePIDInputs.update(intakeSetpoint, intakeEncoderL.get());
    intakeArmMotorL.set(
        (Math.abs(getAngleDifference(getLeftSideAngle(), getRightSideAngle())) < separationConstantThreshold ? 0 : 1)
            * separationConstant * (getAngleDifference(getLeftSideAngle(), getRightSideAngle()) / 360)
            + intakePIDL.calculate(intakeEncoderL.get(), intakeSetpoint)
            - gravityFeedFowardConstant * Math.sin(getLeftSideAngle() * Math.PI / 180));
    intakeArmMotorR.set(
        (Math.abs(getAngleDifference(getLeftSideAngle(), getRightSideAngle())) < separationConstantThreshold ? 0 : 1)
            * separationConstant * (getAngleDifference(getRightSideAngle(), getLeftSideAngle()) / 360)
            + intakePIDR.calculate(getRightAngle(), intakeSetpoint)
            - gravityFeedFowardConstant * Math.sin(getRightSideAngle() * Math.PI / 180));

    NetworkTables.intakeLeftEncoder.setDouble(intakeEncoderL.get());
    NetworkTables.intakeRightEncoder.setDouble(getRightAngle());
    NetworkTables.intakeLeftSideHorizontalAngle.setDouble(getLeftSideAngle());
    NetworkTables.intakeRightSideHorizontalAngle.setDouble(getRightSideAngle());
    gravityFeedFowardConstant = NetworkTables.intakeGravityConstant.getDouble(gravityFeedFowardConstant);
    separationConstant = NetworkTables.intakeSeparationConstant.getDouble(separationConstant);

    armPose = new Pose3d(
        Constants.SIM.intakeMechOffset.getX(),
        Constants.SIM.intakeMechOffset.getY(),
        Constants.SIM.intakeMechOffset.getZ(),
        new Rotation3d(0, Math.toRadians(intakeEncoderL.get() * 360), 0));
    Robot.mecanismPoses[0] = armPose;
  }

  public void addBall(){
    if (simBalls >= 30){
      return;
    }
    simBalls += 1;
  }

  public boolean isFull(){
    return simBalls >= 30;
  }

  public int getBalls(){
    return simBalls;
  }

  public void removeBall(){
    simBalls -= 1;
    if (simBalls < 0){
      simBalls = 0;
    }
  }
}
