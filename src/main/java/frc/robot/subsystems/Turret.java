package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  Drive drive;
  Transform3d turretOffset;
  Transform2d turretTransform2d;
  Pose2d turretPos;

  //An array for fuel that are in the air
  private final List<Fuel> activeFuel = new ArrayList<>();

  //Hub dimensions on the XY plane
  Translation2d hub = new Translation2d(11.915, 4.035);


  public Turret(Drive drive, Transform3d turretOffset) {
    this.drive = drive;
    this.turretOffset = turretOffset;

    turretTransform2d = new Transform2d(turretOffset.getX(), turretOffset.getY(), new Rotation2d());
    turretPos = drive.getPose().plus(turretTransform2d);
  }

  @Override
  public void periodic() {
    //Turret Yaw Visualization
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
    Logger.recordOutput(
        "FinalPoses",
        new Pose3d[] {
          new Pose3d(
              turretOffset.getTranslation(),
              new Rotation3d(
                  0,
                  0,
                  trackHub(
                      drive.getPose())))
        });

    //Removes the fuel once the isDead() function is met, and updates the time for the fuel.
    activeFuel.removeIf(
        fuel -> {
          fuel.update(0.02);
          return fuel.isDead();
        });
    
    //Records the fuel to AdvantageScope
    Logger.recordOutput(
        "GamePieces/Fuel", activeFuel.stream().map(Fuel::getPose).toArray(Pose3d[]::new));
  }

  //Aim to the hub
  public double trackHub(Pose2d robotPose) {
    double deltax = hub.getX() - turretPos.getX();
    double deltay = hub.getY() - turretPos.getY();
    double initTheta;
    // Initial theta
    if (deltax == 0) {
      initTheta = 0;
    } else {
      initTheta = Math.atan(deltay / deltax);
    }

    double theta =
        (initTheta - robotPose.getRotation().getRadians())
            + Math.PI / 2
            + Math.signum(deltax) * Math.PI / 2;

    return theta;
  }

  public double calcVelocity() {
    Translation2d hub = new Translation2d(11.915, 4.035);
    double yf = 1.329;
    double xf =
        Math.sqrt(
            Math.pow((hub.getX() - turretPos.getX()), 2)
                + Math.pow((hub.getY() - turretPos.getY()), 2));
    double g = 9.8;
    double vel = Math.sqrt(
      g*yf + g*Math.sqrt(Math.pow(xf, 2) + Math.pow(yf, 2))
    ) + 0.5;

    return vel;
  }

  public double calcPitch() {
    Translation2d hub = new Translation2d(11.915, 4.035);
    double yf = 1.329;
    double xf =
        Math.sqrt(
            Math.pow((hub.getX() - turretPos.getX()), 2)
                + Math.pow((hub.getY() - turretPos.getY()), 2));
    double v = calcVelocity();
    double g = 9.8;
    double a = -g;

    double A = a * Math.pow(xf, 2) / (2 * Math.pow(v, 2));
    double B = xf;
    double C = A - yf;

    double theta = Math.atan((-B - Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A));

    return theta;
  }

  public void shootBall() {
    activeFuel.add(new Fuel(calcPitch(), calcVelocity(), drive.getPose().plus(turretTransform2d)));
  }

  public Command shootBallCommand() {
    return runOnce(
        () -> {
          shootBall();
        });
  }
}
