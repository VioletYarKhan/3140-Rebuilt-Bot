package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Controller.controllers;
import frc.robot.subsystems.odometry.Odometry;

public class ManualAim extends AimType {

    private final double manualAimRotationSpeed = 240; // degrees per second
    private final double manualAimHoodSpeed = 30; // degrees per second

    private double desiredRotationAngle = 0;

    public ManualAim() {
        desiredRotationAngle = 0;
        rotationAngle = 0;
        hoodAngle = 0;
        flywheelSpeed = 0;
    }

    @Override
    public void periodic(double deltaTime, double hoodMeasurement, double flywheelMeasurement,
            double rotationMeasurement) {
        desiredRotationAngle += -Controller.getInstance().getRightX(controllers.SECONDARY) * manualAimRotationSpeed * deltaTime;
        hoodAngle += -Controller.getInstance().getLeftY(controllers.SECONDARY) * manualAimHoodSpeed * deltaTime;
        hoodAngle = Math.max(Constants.Limits.Turret.minPitch, Math.min(hoodAngle, Constants.Limits.Turret.maxPitch)); 
        rotationAngle = Math.max(-90, Math.min(desiredRotationAngle - Odometry.getInstance().getRotation().getDegrees(), 90));
        if (Math.abs(Controller.getInstance().getRightX(controllers.SECONDARY)) > 0.02 && (desiredRotationAngle < -90 || desiredRotationAngle > 90)) desiredRotationAngle = rotationAngle + Odometry.getInstance().getRotation().getDegrees();

        this.shouldShoot = true;
    }

    @Override
    public void activate(double rotationAngle, double hoodAngle, double flywheelSpeed) {

    }

    @Override
    public void deactivate() {

    }

    @Override
    public double getLookDirection() {
        return desiredRotationAngle;
    }

    public void setDesiredRotationAngle(double angle) {
        desiredRotationAngle = angle;
    }

    public void setHoodAngle(double angle) {
        hoodAngle = angle;
    }
}
