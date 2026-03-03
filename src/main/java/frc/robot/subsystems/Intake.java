// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Robot;

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
import frc.robot.libs.AbsoluteEncoder;

public class Intake extends SubsystemBase {
  private Pose3d armPose = Constants.SIM.intakeMechOffset;

  private SparkMax intakeArmMotor = new SparkMax(Constants.MotorIDs.intakeArmMotor, SparkMax.MotorType.kBrushless);
  private SparkMax intakeArmMotorFollower = new SparkMax(Constants.MotorIDs.intakeArmMotorFollower,
      SparkMax.MotorType.kBrushless);
  private SparkMax intakeRollerMotor = new SparkMax(Constants.MotorIDs.intakeMotor, SparkMax.MotorType.kBrushless);
  public SparkMaxSim intakeArmMotorSim;

  public DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(Constants.SensorIDs.intakeEncoder, 1, 0);
  public DutyCycleEncoderSim intakeEncoderSim = new DutyCycleEncoderSim(intakeEncoder);

  private double intakeSetpoint = 0;

  private PIDController intakePID = new PIDController(
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
    config.idleMode(IdleMode.kBrake);

    intakeArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.follow(intakeArmMotor.getDeviceId(), true);
    intakeArmMotorFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (RobotBase.isSimulation()) {
      intakeArmMotorSim = new SparkMaxSim(intakeArmMotor, DCMotor.getNEO(1));
    }
  }

  public void deploy() {
    intakeSetpoint = Constants.Limits.Intake.deployedPosition;
  }

  public void stow() {
    intakeSetpoint = Constants.Limits.Intake.stowedPosition;
  }

  public boolean isStowed() {
    return intakeSetpoint == Constants.Limits.Intake.stowedPosition;
  }

  public double getAngle() {
    return intakeEncoder.get();
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

  @Override
  public void periodic() {
    intakeArmMotor.set(intakePID.calculate(intakeEncoder.get(), intakeSetpoint));

    armPose = new Pose3d(
        Constants.SIM.intakeMechOffset.getX(),
        Constants.SIM.intakeMechOffset.getY(),
        Constants.SIM.intakeMechOffset.getZ(),
        new Rotation3d(0, Math.toRadians(intakeEncoder.get()), 0));
    Robot.mecanismPoses[0] = armPose;
  }
}
