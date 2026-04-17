// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase {
  private static Climbers m_Instance = null;

  private final SparkMax climberMotor = new SparkMax(Constants.MotorIDs.climberMotor, MotorType.kBrushless);

  private Encoder climberEncoder = new Encoder(Constants.SensorIDs.climberEncoderA, Constants.SensorIDs.climberEncoderB);

  private final SparkMaxConfig config = new SparkMaxConfig();

  public static double extenededPosition = 12;
  public static double extendSpeed = 0.25;
  public static double climbSpeed = -0.5;


  public static Climbers getInstance() {
    if (m_Instance == null) {
      m_Instance = new Climbers();
    }

    return m_Instance;
  }

  /** Creates a new Climber. */
  public Climbers() {
    // Configure Motors
    config.inverted(true).smartCurrentLimit(40).idleMode(IdleMode.kBrake);

    climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climberEncoder.setDistancePerPulse(0.01);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotor(double speed) {
    climberMotor.set(speed);
  }

  public double getEncoderDistance() {
    return climberEncoder.getDistance();
  }
}
