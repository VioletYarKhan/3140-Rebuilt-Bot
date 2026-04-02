package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.AbsoluteEncoder;

public class SwerveModule extends SubsystemBase {
    public String moduleID;
    public int driveMotorID;
    public int turnMotorID;
    public double baseAngle;
    public SparkMax turnMotor;
    public SparkMaxSim simTurnMotor;
    public SparkFlex driveMotor;
    public SparkFlexSim simDriveMotor;
    public PIDController turnPID;
    public ProfiledPIDController drivePID;
    public AbsoluteEncoder turnEncoder;
    public RelativeEncoder driveEncoder;

    public double driveSetpointTolerance = .5;
    public double turnSetpointTolerance = 5;
    public double turnVelocityTolerance = 1;

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.084706 * .712, 2.4433 * .712,
            0.10133 * .712);

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.Bot.maxChassisSpeed,
            Constants.Bot.maxAcceleration);

    // private State initialState = new TrapezoidProfile.State(0, 0);
    // private TrapezoidProfile trapezoidProfile;

    // Conversion Factor for the motor encoder output to wheel output
    // (Circumference / Gear Ratio) * Inches to meters conversion

    public SwerveModule(String moduleID, int analogID, int driveMotorID, int turnMotorID, double baseAngle,
            boolean driveInverted) {
        this.moduleID = moduleID;
        this.baseAngle = baseAngle;
        this.turnMotorID = turnMotorID;
        this.driveMotorID = driveMotorID;

        SparkFlexConfig driveMotorConfig = new SparkFlexConfig();

        driveMotorConfig.idleMode(IdleMode.kBrake).inverted(driveInverted).smartCurrentLimit(60);

        driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig turnMotorConfig = new SparkMaxConfig();

        turnMotorConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(40);

        turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);

        turnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turnEncoder = new AbsoluteEncoder(analogID, baseAngle);

        driveEncoder = driveMotor.getEncoder();

        /////// AI CODE ///////
        // Use simulation-specific PID values if in simulation for smoother response
        if (RobotBase.isSimulation()) {
            turnPID = new PIDController(Constants.Bot.simTurnP, 0, 0);
        } else {
            turnPID = new PIDController(Constants.Bot.turnP, 0, Constants.Bot.turnD);
        }
        /////// END AI CODE ///////

        // we don't use I or D since P works well enough
        turnPID.enableContinuousInput(0, 360);
        turnPID.setTolerance(turnSetpointTolerance, turnVelocityTolerance);

        /////// AI CODE ///////
        // determined from a SYSID scan
        if (RobotBase.isSimulation()) {
            drivePID = new ProfiledPIDController(Constants.Bot.simDriveP, Constants.Bot.simDriveI,
                    Constants.Bot.simDriveD, constraints);
        } else {
            drivePID = new ProfiledPIDController(0.005, 0, 0.0005, constraints);
        }
        /////// END AI CODE ///////
        drivePID.setConstraints(constraints);
        drivePID.setTolerance(driveSetpointTolerance);

        if (RobotBase.isSimulation()) {
            simDriveMotor = new SparkFlexSim(driveMotor, DCMotor.getNeoVortex(1));

            simTurnMotor = new SparkMaxSim(turnMotor, DCMotor.getNEO(1));
        }
    }

    // runs while the bot is running
    @Override
    public void periodic() {
        // setAngle(0);
        // turnPID.calculate(getTurnEncoder().getAbsolutePosition());
        // Fixed: Removed accelerationLimiter reset - it was resetting to 0 every cycle
        // causing progressive slowdown
        // Fixed: Removed constraint recreation - it's already set in constructor and
        // doesn't need to be recreated
        NetworkTableInstance.getDefault().getTable("Angle").getEntry(moduleID)
                .setDouble(turnEncoder.getAbsolutePosition());
    }

    SlewRateLimiter accelerationLimiter = new SlewRateLimiter(Constants.Bot.maxAcceleration,
            -Constants.Bot.maxAcceleration, 0);

    /////// AI CODE ///////
    // Simulation-only momentum tracking
    private double simCurrentSpeed = 0; // Current simulated wheel speed
    private double simPreviousMeasuredVelocity = 0; // For velocity smoothing
    /////// END AI CODE ///////

    public void setStates(SwerveModuleState state) {
        double currentAngle = turnEncoder.getAbsolutePosition();

        NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/CurrentAngle")
                .setDouble(currentAngle);
        NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/DesiredAngleBeforeOptimize")
                .setDouble(state.angle.getDegrees());
        NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/DesiredSpeedBeforeOptimize")
                .setDouble(state.speedMetersPerSecond);

        double angle;
        if (RobotBase.isSimulation()) {
            angle = (currentAngle + 180) % 360;
            if (angle < 0) {
                angle += 360;
            }
        } else {
            angle = currentAngle;
        }

        state.optimize(new Rotation2d(Units.degreesToRadians(angle)));

        NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/DesiredAngleAfterOptimize")
                .setDouble(state.angle.getDegrees());
        NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/DesiredSpeedAfterOptimize")
                .setDouble(state.speedMetersPerSecond);

        setAngle(state.angle.getDegrees());
        setDriveSpeed(accelerationLimiter.calculate(state.speedMetersPerSecond));

        NetworkTableInstance.getDefault().getTable("Speed").getEntry(moduleID).setDouble(state.speedMetersPerSecond);

        if (RobotBase.isSimulation()) {
            NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/TurnMotorPos")
                    .setDouble(simTurnMotor.getPosition());
            NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/DriveMotorRPM")
                    .setDouble(simDriveMotor.getVelocity());
        }
    }

    public void setAngle(double angle) {
        turnPID.setSetpoint(angle);
        turnMotor.set(-turnPID.calculate(turnEncoder.getAbsolutePosition()));
    }

    public void setDriveSpeed(double velocity) {
        // velocity is desired wheel speed in meters/second
        /////// AI CODE ///////
        double targetVelocity = velocity;

        // Apply momentum simulation if in simulation mode
        if (RobotBase.isSimulation()) {
            targetVelocity = applySimulationMomentum(velocity);
        }
        /////// END AI CODE ///////

        drivePID.setGoal(new State(targetVelocity, 0));

        // driveEncoder.getVelocity() returns RPM -> convert to meters/sec:
        // measuredVelocity = (RPM / 60) * metersPerWheelRotation
        double measuredVelocity = driveEncoder.getVelocity() * Constants.Bot.encoderRotationToMeters / 60.0;

        /////// AI CODE ///////
        // Apply velocity smoothing in simulation to reduce oscillation
        if (RobotBase.isSimulation()) {
            // Exponential moving average with alpha = 0.3 (70% previous, 30% new)
            measuredVelocity = 0.7 * simPreviousMeasuredVelocity + 0.3 * measuredVelocity;
            simPreviousMeasuredVelocity = measuredVelocity;
        }
        /////// END AI CODE ///////

        // Feedforward (expects m/s) + PID (measurement in m/s)
        double voltage = driveFeedforward.calculate(targetVelocity) + drivePID.calculate(measuredVelocity);

        driveMotor.setVoltage(voltage);

        NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Set Speed").setDouble(targetVelocity);
        NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Actual Speed").setDouble(measuredVelocity);
    }

    /////// AI CODE ///////
    /**
     * Applies momentum simulation for realistic coast-down behavior (simulation
     * only)
     * 
     * @param desiredSpeed Target speed commanded by driver
     * @return Speed after applying momentum/deceleration limits
     */
    private double applySimulationMomentum(double desiredSpeed) {
        double speedDifference = desiredSpeed - simCurrentSpeed;
        double dt = 0.02; // 20ms loop time

        if (Math.abs(desiredSpeed) > Math.abs(simCurrentSpeed)) {
            // Accelerating - use fast acceleration with limiter
            double limitedSpeed = accelerationLimiter.calculate(desiredSpeed);
            simCurrentSpeed = limitedSpeed;
        } else {
            // Decelerating - apply slow deceleration + drag
            double maxDecelStep = Constants.Bot.simMaxDeceleration * dt;
            double dragForce = simCurrentSpeed * Constants.Bot.simDragCoefficient;

            // Move toward desired speed, but limited by decel rate + drag
            double decelStep = Math.min(Math.abs(speedDifference), maxDecelStep);
            simCurrentSpeed -= Math.signum(simCurrentSpeed) * (decelStep + Math.abs(dragForce));

            // Clamp to zero at very low speeds to avoid drift
            if (Math.abs(simCurrentSpeed) < 0.01) {
                simCurrentSpeed = 0;
            }

            // If we've reached the desired speed, clamp to it (but only if moving in same
            // direction)
            if (Math.abs(simCurrentSpeed - desiredSpeed) < 0.01) {
                simCurrentSpeed = desiredSpeed;
            }
        }

        return simCurrentSpeed;
    }
    /////// END AI CODE ///////

    public void setTurnSpeed(double speed) {
        speed = Math.max(Math.min(speed, Constants.Bot.maxTurnSpeed), -Constants.Bot.maxTurnSpeed);
        turnMotor.set(speed);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        double angle = turnEncoder.getAbsolutePosition();
        // driveEncoder.getPosition() is in motor rotations -> encoderRotationToMeters
        // is meters per wheel rotation.
        // If encoderRotationToMeters is meters per wheel rotation, ensure
        // encoder.getPosition() is wheel rotations
        // (if not, make sure your constant matches motor->wheel ratio). This keeps
        // behavior identical to real robot.
        double distance = driveEncoder.getPosition() * Constants.Bot.encoderRotationToMeters;
        return new SwerveModulePosition(distance, new Rotation2d(Units.degreesToRadians(angle)));
    }

    public double getVelocity() {
        // Use driveEncoder (RPM) -> convert to meters/sec
        double measuredVelocity = driveEncoder.getVelocity() * Constants.Bot.encoderRotationToMeters / 60.0;
        return measuredVelocity;
    }

    public AbsoluteEncoder getTurnEncoder() {
        return this.turnEncoder;
    }

    public String getModuleID() {
        return this.moduleID;
    }

    public SwerveModuleState getState() {
        // Return module state with wheel speed in meters/sec and module angle in
        // radians
        double speedMetersPerSecond = driveEncoder.getVelocity() * Constants.Bot.encoderRotationToMeters / 60.0;
        return new SwerveModuleState(speedMetersPerSecond,
                Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
    }
}
