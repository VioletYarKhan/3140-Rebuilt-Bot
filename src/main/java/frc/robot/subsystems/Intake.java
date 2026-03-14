// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.libs.NetworkTables;
import frc.robot.subsystems.Turret.TurretMain;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
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

  public DutyCycleEncoder intakeEncoderL = new DutyCycleEncoder(Constants.SensorIDs.intakeEncoderL, 1, 0);
  public DutyCycleEncoder intakeEncoderR = new DutyCycleEncoder(Constants.SensorIDs.intakeEncoderR, 1, 0);
  public DutyCycleEncoderSim intakeEncoderSim = new DutyCycleEncoderSim(intakeEncoderL);

  public double rightEncoderOffset = 0.93-(1-0.76);

  private double intakeSetpoint = Constants.Limits.Intake.stowedPosition;

  private TurretMain.LoggedPIDInputs intakePIDInputs = new TurretMain.LoggedPIDInputs(
    "Intake",
      Constants.PID.Intake.intakeP,
      Constants.PID.Intake.intakeI,
      Constants.PID.Intake.intakeD
      );
  
  private PIDController intakePIDL = new PIDController(
      Constants.PID.Intake.intakeP,
      Constants.PID.Intake.intakeI,
      Constants.PID.Intake.intakeD
      );
  private PIDController intakePIDR = new PIDController(
      Constants.PID.Intake.intakeP,
      Constants.PID.Intake.intakeI,
      Constants.PID.Intake.intakeD
      );

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
    

    intakeRollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(true);


    config.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(15);
    intakeArmMotorL.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(false);
    intakeArmMotorR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    if (RobotBase.isSimulation()) {
      intakeArmMotorSim = new SparkMaxSim(intakeArmMotorL, DCMotor.getNEO(1));
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
    return intakeEncoderL.get();
  }

  public double getRightAngle() {
    return ((1 - intakeEncoderR.get()) + rightEncoderOffset) % 1;
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

  @Override
  public void periodic() {
    intakePIDL.setP(intakePIDInputs.getP());
    intakePIDL.setI(intakePIDInputs.getI());
    intakePIDL.setD(intakePIDInputs.getD());
    intakePIDR.setP(intakePIDInputs.getP());
    intakePIDR.setI(intakePIDInputs.getI());
    intakePIDR.setD(intakePIDInputs.getD());

    intakePIDInputs.update(intakeSetpoint, intakeEncoderL.get());
    intakeArmMotorL.set(intakePIDL.calculate(intakeEncoderL.get(), intakeSetpoint));
    intakeArmMotorR.set(intakePIDR.calculate(getRightAngle(), intakeSetpoint));

    NetworkTables.intakeLeftEncoder.setDouble(intakeEncoderL.get());
    NetworkTables.intakeRightEncoder.setDouble(getRightAngle());

    armPose = new Pose3d(
        Constants.SIM.intakeMechOffset.getX(),
        Constants.SIM.intakeMechOffset.getY(),
        Constants.SIM.intakeMechOffset.getZ(),
        new Rotation3d(0, Math.toRadians(intakeEncoderL.get()), 0));
    Robot.mecanismPoses[0] = armPose;
  }
}
