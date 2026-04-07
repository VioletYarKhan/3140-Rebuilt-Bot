// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.libs.NetworkTables;
import frc.robot.subsystems.Turret.ManualAim;
import frc.robot.subsystems.Turret.TurretMain;
import frc.robot.subsystems.Turret.TurretMain.AimOpt;

public class Controller extends SubsystemBase {
  private static Controller instance = null;

  public final XboxController primaryController;

  public final XboxController secondaryController;

  static boolean linuxSim = Robot.isSimulation();

  enum AxisMapping {
    LEFT_X(linuxSim ? 0 : 0),
    RIGHT_X(linuxSim ? 3 : 4),
    LEFT_Y(linuxSim ? 1 : 1),
    RIGHT_Y(linuxSim ? 4 : 5),
    LEFT_TRIGGER(linuxSim ? 2 : 2),
    RIGHT_TRIGGER(linuxSim ? 5 : 3);

    public int axisValue;

    AxisMapping(int axisValue) {
      this.axisValue = axisValue;
    }
  }

  /** Creates a new Controller. */
  public Controller(int primary, int secondary) {
    primaryController = new XboxController(primary);
    secondaryController = new XboxController(secondary);
  }

  private final double deadband = 0.1;
  private final double triggerThreshold = 0.5;

  private final boolean testing = false;

  public enum controllers {
    PRIMARY, SECONDARY
  }

  public enum ControlMode {
    AUTO, MANUAL, OHNO_MANUAL
  }

  private ControlMode curControlMode = ControlMode.AUTO;

  public static Controller getInstance() {
    if (instance == null) {
      instance = new Controller(Constants.Controller.DriverControllerPort,
          Constants.Controller.SecondaryDriverControllerPort);
    }
    return instance;
  }

  public boolean getLeftTriggerTriggered(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    return contr.getRawAxis(AxisMapping.LEFT_TRIGGER.axisValue) > triggerThreshold;
  }

  public boolean getRightTriggerTriggered(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);

    return contr.getRawAxis(AxisMapping.RIGHT_TRIGGER.axisValue) > triggerThreshold;
  }

  public double getLeftX(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    double x = contr.getRawAxis(AxisMapping.LEFT_X.axisValue);
    if (Math.abs(x) > deadband) {
      if (x > 0)
        return Math.pow(x, 2);
      else
        return -Math.pow(x, 2);

    } else {
      return 0;
    }
  }

  public double getRightX(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    double x = contr.getRawAxis(AxisMapping.RIGHT_X.axisValue);
    if (Math.abs(x) > deadband) {
      if (x > 0)
        return Math.pow(x, 2);
      else
        return -Math.pow(x, 2);
    } else {
      return 0;
    }
  }

  public double getLeftY(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    double y = contr.getRawAxis(AxisMapping.LEFT_Y.axisValue);
    if (Math.abs(y) > deadband) {
      if (y > 0)
        return Math.pow(y, 2);
      else
        return -Math.pow(y, 2);

    } else {
      return 0;
    }
  }

  public double getRightY(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    double y = contr.getRawAxis(AxisMapping.RIGHT_Y.axisValue);
    if (Math.abs(y) > deadband) {
      if (y > 0)
        return Math.pow(y, 2);
      else
        return -Math.pow(y, 2);
    } else {
      return 0;
    }
  }

  public void setRumbleBoth(controllers controller, double duration, double intensity) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              contr.setRumble(RumbleType.kBothRumble, intensity);
            }),
            new WaitCommand(duration)),
        new InstantCommand(() -> {
          contr.setRumble(RumbleType.kBothRumble, 0);
        })).schedule();
  }

  public void setRumbleLeft(controllers controller, double duration, double intensity) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              contr.setRumble(RumbleType.kLeftRumble, intensity);
            }),
            new WaitCommand(duration)),
        new InstantCommand(() -> {
          contr.setRumble(RumbleType.kBothRumble, 0);
        })).schedule();
  }

  public void setRumbleRight(controllers controller, double duration, double intensity) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              contr.setRumble(RumbleType.kRightRumble, intensity);
            }),
            new WaitCommand(duration)),
        new InstantCommand(() -> {
          contr.setRumble(RumbleType.kBothRumble, 0);
        })).schedule();
  }

  public void setControlMode(ControlMode mode) {
    curControlMode = mode;
  }

  public ControlMode getControlMode() {
    return curControlMode;
  }

  /**
   * Reusable controls for all modes go here.
   * 
   * Mapped Buttons:
   *
   * PRIMARY CONTROLLER:
   * Y - Reset Gyro
   * X - Toggle Intake Stowed/Deployed
   * A - Hold to go to feed Position and release to deploy
   * B - Lock Wheels
   * Left Bumper - Intake
   * Right Bumper - Feed
   * Left Trigger - Reverse Feeder
   * Right Trigger - Spinup Flywheel
   *
   * SECONDARY CONTROLLER:
   * Manual Mode only:
   * Right Bumper - Hold for shoot setpoint
   * Left Bumper - Hold for pickup setpoint
   *
   * Left Stick - Hood
   * Right Stick - Turret Rotation
   *
   * A - Set Turret Rotation to 0
   * B - Set Hood Angle to 0
   *
   * Y - Recalibrate Camera Pose
   */
  private void reusableDefaultControls() {
    if (primaryController.getYButtonPressed())
      SwerveDrive.odometry.resetGyro();

    if (secondaryController.getYButtonPressed())
      RobotContainer.odometry.recalibrateCameraPose();

    Intake.getInstance()
        .intake(primaryController.getLeftBumperButton()
            ? (NetworkTables.intakeRollerSpeed_d.getDouble(Constants.MotorSpeeds.Intake.intakeSpeed))
            : (primaryController.getPOV() == 0 ? Constants.MotorSpeeds.Intake.outtakeSpeed : 0));

    TurretMain.getInstance().setFlywheelActive(getRightTriggerTriggered(controllers.PRIMARY));

    if (primaryController.getXButtonPressed()) {
      if (Intake.getInstance().isStowed()) {
        Intake.getInstance().deploy();
      } else {
        Intake.getInstance().stow();
      }
    }

    if (primaryController.getAButtonPressed()) {
      Intake.getInstance().feed();
    } else if(primaryController.getAButton()) {
      if(Math.abs(Math.abs(Intake.getInstance().getAngle()) - Constants.Limits.Intake.deployedPosition) < (15.0 / 360.0)) {
        System.out.println("feed"); 
        Intake.getInstance().feed();
      }
      if(Math.abs(Math.abs(Intake.getInstance().getAngle()) - Constants.Limits.Intake.feedPosition) < (5.0 / 360.0))  {
        System.out.println("deploy"); 
        Intake.getInstance().deploy();
      }
    } else if (primaryController.getAButtonReleased()) {
      Intake.getInstance().deploy();
    }

    if (primaryController.getBButtonPressed()) {
      Robot.locked = true;
    } else if (primaryController.getBButtonReleased()) {
      Robot.locked = false;
    }

    Feeder.getInstance().setFeederActive(primaryController.getRightBumperButton()/*
                                                                                  * && (TurretMain.getInstance().
                                                                                  * shouldShoot() ||
                                                                                  * !TurretMain.getInstance().
                                                                                  * getFlywheelActive())
                                                                                  */);
    Feeder.getInstance().setFeederInverted(!TurretMain.getInstance().getFlywheelActive());
  }

  private void updateControlMode() {
    if (secondaryController.getRightBumperButtonPressed()) {

      TurretMain.getInstance().setAimMode(AimOpt.MANUAL);

      curControlMode = ControlMode.MANUAL;
      System.out.println("Control Mode: " + curControlMode);
      setRumbleBoth(controllers.SECONDARY, 0.1, 1);
    } else if (secondaryController.getLeftBumperButtonPressed()) {

      TurretMain.getInstance().setAimMode(AimOpt.AUTO);

      curControlMode = ControlMode.AUTO;
      System.out.println("Control Mode: " + curControlMode);
      setRumbleBoth(controllers.SECONDARY, 0.1, 1);
    } else if (getLeftTriggerTriggered(controllers.SECONDARY) && getRightTriggerTriggered(controllers.SECONDARY)) {

      TurretMain.getInstance().setAimMode(AimOpt.MANUAL);

      curControlMode = ControlMode.OHNO_MANUAL;
      System.out.println("Control Mode: " + curControlMode);
      setRumbleBoth(controllers.SECONDARY, 0.1, 1);
    }
  }

  private void AutoMode() {
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      updateControlMode();
      return;
    }

    reusableDefaultControls();
  }

  private void ManualMode() {
    ManualAim manualAim = (ManualAim) (TurretMain.getInstance().aimTypes.get(TurretMain.AimOpt.MANUAL));
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      updateControlMode();
      return;
    }

    NetworkTables.flywheelRPMOverride_d.setDouble(5000);
    reusableDefaultControls();

    TurretMain.flywheelRPMOverride = true;
    if (secondaryController.getRightBumperButton()) {
      TurretMain.hoodAngleOverride = true;
      TurretMain.flywheelRPMOverride = true;
      NetworkTables.hoodAngle_d.setDouble(25);
      NetworkTables.flywheelRPMOverride_d.setDouble(5000);
    } else {
      TurretMain.hoodAngleOverride = false;
      TurretMain.flywheelRPMOverride = false;
    }

    if (secondaryController.getLeftBumperButton()) {
      TurretMain.hoodAngleOverride = true;
      TurretMain.flywheelRPMOverride = true;
      NetworkTables.hoodAngle_d.setDouble(35); // TODO: Tune values for passing
      NetworkTables.flywheelRPMOverride_d.setDouble(6700); // TODO: Tune values for passing
    } else if (!secondaryController.getRightBumperButton()) {
      TurretMain.hoodAngleOverride = false;
      TurretMain.flywheelRPMOverride = false;
    }

    if (secondaryController.getAButton()) {
      manualAim.setDesiredRotationAngle(0);
    }
    if (secondaryController.getBButton()) {
      manualAim.setHoodAngle(0);
    }
  }

  private void OHNOManualMode() {
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      updateControlMode();
      return;
    }
    Feeder.getInstance().setFeederActive(
        primaryController.getRightTriggerAxis() > triggerThreshold && primaryController.getRightBumperButton());
  }

  private void testingMode() {
    System.out.println("Testing Mode Active - Controller Periodic Function");
  }

  public void periodic() {
    if (DriverStation.isAutonomous())
      return;
    NetworkTables.driveModeManual_b.setBoolean(curControlMode == ControlMode.MANUAL);
    if (testing) {
      testingMode();
      return;
    }

    switch (curControlMode) {
      case AUTO:
        NetworkTables.driveModeManual_b.setBoolean(false);
        AutoMode();
        break;
      case MANUAL:
        NetworkTables.driveModeManual_b.setBoolean(true);
        ManualMode();
        break;
      case OHNO_MANUAL:
        NetworkTables.driveModeManual_b.setBoolean(true);
        OHNOManualMode();
        break;
      default:
        // Integral to the code base DO NOT CHANGE! (Copilot did it!)
        throw new IllegalStateException("Invalid control mode: \n Nuh uh, no way, not gonna happen");
    }
  }
}
