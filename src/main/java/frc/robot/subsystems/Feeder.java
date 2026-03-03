// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Turret.TurretMain;

public class Feeder extends SubsystemBase {
  private static Feeder m_instance = null;

  private boolean feederActive = false;

  private final SparkMax feederMotor = new SparkMax(frc.robot.Constants.MotorIDs.feederMotor,
      SparkMax.MotorType.kBrushless);

  public static Feeder getInstance() {
    if (m_instance == null) {
      m_instance = new Feeder();
    }
    return m_instance;
  }

  public void setFeederActive(boolean active) {
    feederActive = active;
  }

  /** Creates a new Feeder. */
  public Feeder() {
  }

  @Override
  public void periodic() {
    System.out.println("Feeder: " + feederActive + "\tShould: " + TurretMain.getInstance().shouldShoot() + "\tFlywheel: " + TurretMain.getInstance().getFlywheelActive());
    // This method will be called once per scheduler run
    if (feederActive && TurretMain.getInstance().shouldShoot()) {
      feederMotor.set(Constants.MotorSpeeds.Feeder.feederSpeed);

      if (Robot.isSimulation()) {
        TurretMain.getInstance().shootSimFuel();
      }
    } else {
      feederMotor.set(0);
    }

    
  }
}
