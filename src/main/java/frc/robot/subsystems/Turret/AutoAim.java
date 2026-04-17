package frc.robot.subsystems.Turret;

import java.util.Optional;

//import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.libs.FlipPose;
import frc.robot.libs.NetworkTables;
import frc.robot.libs.ShotPredictor;
import frc.robot.libs.ShotPredictor.Result;
import frc.robot.libs.Vector2;
import frc.robot.subsystems.odometry.Odometry;

public class AutoAim extends AimType {
  private static double predictForwardTime = Robot.isReal() ? 0.4 : 0.0;
  private static double predictForwardWhenCheckingMultiplier = 1; // Multiplies predict forward time when checking if
                                                                  // a shot will go in for should shoot
                                                                  // private LoggedNetworkNumber errorLog = new
                                                                  // LoggedNetworkNumber("AutoAim/error", 0);

  public static class Target {
    private Vector2 position;
    public double targetHeight;
    public boolean flipIfRed;
    public ShotPredictor.HeightBounds heightBounds;
    public double scoreTolerance;

    public Target(Vector2 position, double targetHeight, boolean flipIfRed,
        ShotPredictor.HeightBounds heightBounds, double scoreTolerance) {
      this.position = position;
      this.targetHeight = targetHeight;
      this.flipIfRed = flipIfRed;
      this.heightBounds = heightBounds;
      this.scoreTolerance = scoreTolerance;
    }

    public Vector2 getPosition() {
      return flipIfRed ? FlipPose.flipVectorIfRed(position) : position;
    }
  }

  private Target hubTarget = new Target(
      new Vector2(4.625, 4.025),
      Units.inchesToMeters(Constants.PathplannerConstants.TopOfHubHeightInches - 5),
      true,
      new ShotPredictor.HeightBounds(
          Units.inchesToMeters(21), // radius of hub top (flat side to flat side of hexagon)
                                    // p
          Units.inchesToMeters(10 + Constants.PathplannerConstants.TopOfHubHeightInches
              + Constants.PathplannerConstants.FuelRadiusInches), // desired height
          Units.inchesToMeters(5 + Constants.PathplannerConstants.TopOfHubHeightInches
              + Constants.PathplannerConstants.FuelRadiusInches) // min height
      ),
      Units.inchesToMeters(21 - 5) // radius of hub top but less forgiving
  );
  private Target leftZoneTarget = new Target(
      new Vector2(2.065, 6.219),
      Units.inchesToMeters(Constants.PathplannerConstants.FuelRadiusInches),
      true,
      new ShotPredictor.HeightBounds(
          2.6, // meters from score point to place we have to shoot over
          3, // desired meter height
          1 // minimum height in meters
      ),
      1 // meters of leeway (radius)
  );
  private Target rightZoneTarget = new Target(
      new Vector2(2.065, 1.787),
      Units.inchesToMeters(Constants.PathplannerConstants.FuelRadiusInches),
      true,
      new ShotPredictor.HeightBounds(
          2.6, // meters from score point to place we have to shoot over
          3, // desired meter height
          1 // minimum height in meters
      ),
      1 // meters of leeway (radius)
  );
  private Target corralRightTarget = new Target(
      new Vector2(0, 0.675),
      Units.inchesToMeters(Constants.PathplannerConstants.FuelRadiusInches),
      true,
      new ShotPredictor.HeightBounds(
          4.7, // meters from score point to place we have to shoot over
          2.5, // desired meter height
          1 // minimum height in meters
      ),
      1 // I loosened the accuracy requirement so its ok to miss // 0.3 // meters of
        // leeway (radius)
  );
  private Vector2 neutralDeadZoneMin = new Vector2(5.239, 3.434);
  private Vector2 neutralDeadZoneMax = new Vector2(6.22, 4.572);

  private Optional<Target> currentTargetOpt = Optional.empty();
  private ShotPredictor shotPredictor;

  public AutoAim(double minPitch, double maxPitch) {
    rotationAngle = 0;
    hoodAngle = 0;
    flywheelSpeed = 0;
    shotPredictor = new ShotPredictor(
        minPitch,
        maxPitch,
        Constants.Limits.Turret.maxAngularVelocity,
        hubTarget.heightBounds);
  }

  private Pair<Pose2d, Vector2> getFutureState(double dt) {
    Odometry odometry = Odometry.getInstance();
    Vector2 futureBotVelocity = odometry.getBotVelocity(true).add(odometry.getBotAcceleration().mult(dt));

    // position = velocity*time + acceleration*0.5*time^2
    Vector2 predictedPositionChange = odometry.getBotVelocity(true).mult(dt).add(
        odometry.getBotAcceleration().mult(0.5 * dt * dt));
    Vector2 futureBotPosition = odometry.getPosition().add(predictedPositionChange);

    Vector2 turretPosition = Constants.PathplannerConstants.botTurretOffset;

    double futureRotation = odometry.getAngle() + odometry.getAngularVelocity() * dt; // in rads
    Vector2 rotatedTurretPosition = turretPosition.rotate(futureRotation);

    Vector2 centripetalVelocity = new Vector2(-rotatedTurretPosition.Y, rotatedTurretPosition.X)
        .mult(odometry.getAngularVelocity());
    Vector2 futureTurretVelocity = futureBotVelocity.add(centripetalVelocity);
    Vector2 shotOrigin = futureBotPosition.add(rotatedTurretPosition);

    return new Pair<Pose2d, Vector2>(new Pose2d(shotOrigin.X, shotOrigin.Y, new Rotation2d(futureRotation)),
        futureTurretVelocity);
  }

  private double getBotVerticalVelocity() {
    return 0; // TODO: (optional) make this work
  }

  private double getTurretHeight() {
    return Units.inchesToMeters(18); // TODO: (optional) offset this up as we climb
  }

  private boolean predict(double deltaTime) {
    if (currentTargetOpt.isEmpty()) {
      return false;
    }

    Target currentTarget = currentTargetOpt.get();

    shotPredictor.Bounds = currentTarget.heightBounds; // incase we want to change desired velocity

    double turretHeight = getTurretHeight();

    Pair<Pose2d, Vector2> futureStatePair = getFutureState(predictForwardTime);
    Pose2d futurePose = futureStatePair.getFirst();
    Vector2 futureShotOrigin = new Vector2(futurePose.getX(), futurePose.getY());
    double futureRotation = futurePose.getRotation().getRadians();
    Vector2 futureTurretVelocity = futureStatePair.getSecond();

    Vector2 relativeTargetPosition = currentTarget.getPosition().sub(futureShotOrigin);

    // boolean isShooting =
    // Controller.getInstance().primaryController.getRightBumperButton();
    double verticalVelocity = getBotVerticalVelocity();
    Optional<Result> result = shotPredictor.Update(
        true, // !isShooting,
        hoodAngle,
        deltaTime,
        futureTurretVelocity,
        verticalVelocity,
        relativeTargetPosition,
        currentTarget.targetHeight,
        turretHeight,
        Constants.Limits.Turret.maxFuelVelocity);

    if (result.isPresent()) {
      hoodAngle = result.get().ShotAngle;
      flywheelSpeed = result.get().ShotSpeed; // Convereted to RPM in TurretMain
      rotationAngle = Math
          .toDegrees(Math.atan2(result.get().AimPosition.Y, result.get().AimPosition.X) - futureRotation);
    }

    return result.isPresent();
  }

  private Pair<Boolean, Double> EstimateWillScore(double hoodMeasurement, double flywheelMeasurement,
      double rotationMeasurement) {
    if (currentTargetOpt.isEmpty()) {
      return new Pair<Boolean, Double>(false, Double.MAX_VALUE);
    }

    Target currentTarget = currentTargetOpt.get();

    // System.out.println("Flywheel: " + flywheelMeasurement);
    Pair<Pose2d, Vector2> futureState = getFutureState(predictForwardTime * predictForwardWhenCheckingMultiplier);
    Pose2d pose = futureState.getFirst();

    Vector2 initialPosition = new Vector2(pose.getX(), pose.getY());
    double initialHeight = getTurretHeight();
    double rotation = pose.getRotation().getRadians();

    Vector2 inheritedVelocity = futureState.getSecond();

    Vector2 horizontalVelocity = new Vector2(Math.cos(Math.toRadians(hoodMeasurement)) * flywheelMeasurement, 0)
        .rotate(rotation + Math.toRadians(rotationMeasurement))
        .add(inheritedVelocity);
    double verticalVelocity = Math.sin(Math.toRadians(hoodMeasurement)) * flywheelMeasurement
        + getBotVerticalVelocity();

    double relativeTargetHeight = currentTarget.targetHeight - initialHeight;
    Vector2 targetPosition = currentTarget.getPosition();

    // d_y = v_0*t + 0.5*g*t^2
    // 0.5*g*t^2 + v_0*t - d_y = 0
    // (-b +- sqrt(b^2 - 4ac))/(2a)
    double a = 0.5 * ShotPredictor.gravity;
    double b = verticalVelocity;
    double c = -relativeTargetHeight;
    double desc = b * b - 4 * a * c;

    if (desc < 0) {
      //System.out.println("Never reaches height!");
      //return new Pair<Boolean, Double>(false, Double.MAX_VALUE);
    }

    // the times that the ball is at the right height
    double[] ScoreTs = { (-b + Math.sqrt(desc)) / (2 * a), (-b - Math.sqrt(desc)) / (2 * a) };

    double bestDistSq = Double.MAX_VALUE;
    Optional<Double> bestT = Optional.empty();

    // check which t is most likely to be when it scores by checking horizontal
    // distance
    for (int i = 0; i < ScoreTs.length; i++) {
      double t = ScoreTs[i];

      if (t <= 0) {
        continue;
      }

      double scoreDistSq = horizontalVelocity.mult(t).add(initialPosition).sub(targetPosition).magSq();
      // System.out.println("Score dist: " + Math.sqrt(scoreDistSq));
      if (scoreDistSq > currentTarget.scoreTolerance * currentTarget.scoreTolerance) {
        if (scoreDistSq < bestDistSq) {
          bestDistSq = scoreDistSq;
        }
        continue;
      }

      if (scoreDistSq < bestDistSq) {
        bestDistSq = scoreDistSq;
        bestT = Optional.of(t);
      }
    }

    if (bestT.isEmpty()) {
      //System.out.println("No good score time!");
      return new Pair<Boolean, Double>(false, bestDistSq);
    }

    double scoreT = bestT.get();

    Vector2 deltaPosition = targetPosition.sub(initialPosition);
    double checkDist = currentTarget.heightBounds.DistanceFromTarget;
    // dist^2 = (dx - vx*t)^2 + (dy - vy*t)^2
    // dist^2 = dx^2 - 2*dx*vx*t + vx^2*t^2 + dy^2 - 2*dy*vy*t + vy^2*t^2
    // 0 = (dx^2 + dy^2 - dist^2) - (2*dx*vx + 2*dy*vy) * t + (vx^2 + vy^2)*t^2
    a = horizontalVelocity.X * horizontalVelocity.X + horizontalVelocity.Y * horizontalVelocity.Y;
    b = -2 * (deltaPosition.X * horizontalVelocity.X + deltaPosition.Y * horizontalVelocity.Y);
    c = deltaPosition.X * deltaPosition.X + deltaPosition.Y * deltaPosition.Y - checkDist * checkDist;
    desc = b * b - 4 * a * c;

    if (desc < 0) {
      System.out.println("Never enters ring!");
      return new Pair<Boolean, Double>(false, Math.sqrt(bestDistSq));
    }

    // the times that the ball passes the height check radius for the target
    double[] CheckTs = { (-b - Math.sqrt(desc)) / (2 * a), (-b + Math.sqrt(desc)) / (2 * a) };

    // succeed if the ball passes over the check radius, and all times that it does
    // before scoring, its in the proper height bounds
    boolean success = false;
    for (int i = 0; i < CheckTs.length; i++) {
      double t = CheckTs[i];

      if (t <= 0 || t > scoreT) {
        continue;
      }

      double height = initialHeight + verticalVelocity * t + 0.5 * ShotPredictor.gravity * t * t;
      if (!currentTarget.heightBounds.IsInBounds(height)) {
        System.out.println("Passes ring out of height bounds!!");
        return new Pair<Boolean, Double>(false, Math.sqrt(bestDistSq));
      }

      success = true;
    }

    return new Pair<Boolean, Double>(success, Math.sqrt(bestDistSq));
  }

  public void updateTarget() {
    Vector2 botPos = Odometry.getInstance().getPosition();
    Vector2 flippedBotPos = FlipPose.flipVectorIfRed(botPos);

    if (flippedBotPos.isInBounds(neutralDeadZoneMin, neutralDeadZoneMax)) {
      currentTargetOpt = Optional.empty();
      return;
    }

    if (flippedBotPos.X <= Constants.PathplannerConstants.neutralZoneDivision) {
      currentTargetOpt = Optional.of(hubTarget);
      return;
    }

    if (flippedBotPos.Y < Constants.PathplannerConstants.middleY) {
      currentTargetOpt = Optional.of(
          NetworkTables.autoAimForCorral_b.getBoolean(true) ? corralRightTarget : rightZoneTarget);
      return;
    }

    currentTargetOpt = Optional.of(leftZoneTarget);
  }

  @Override
  public void periodic(double deltaTime, double hoodMeasurement, double flywheelMeasurement,
      double rotationMeasurement) {
    updateTarget();

    Boolean predictShouldShoot = predict(deltaTime);
    if (!predictShouldShoot) {
      shouldShoot = false;
    } else {
      Pair<Boolean, Double> resultPair = EstimateWillScore(hoodMeasurement, flywheelMeasurement, rotationMeasurement);
      shouldShoot = resultPair.getFirst();
    }

  }

  @Override
  public void activate(double rotationAngle, double hoodAngle, double flywheelSpeed) {
    this.rotationAngle = rotationAngle;
    this.hoodAngle = hoodAngle;
    this.flywheelSpeed = flywheelSpeed;
  }

  @Override
  public void deactivate() {

  }

  @Override
  public double getLookDirection() {
    if (currentTargetOpt.isEmpty()) {
      return Odometry.getInstance().getRotation().getDegrees();
    }

    Target currentTarget = currentTargetOpt.get();

    Vector2 relTargetPos = currentTarget.getPosition().sub(Odometry.getInstance().getPosition());
    return Math.toDegrees(Math.atan2(relTargetPos.Y, relTargetPos.X));
  }
}
