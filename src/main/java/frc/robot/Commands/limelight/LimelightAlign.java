package frc.robot.commands.limelight;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LimeLightAlign extends CommandBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private final double targetMeters = 0.9144; // 3 ft in meters
    private final double kP_DIST = 1.0;   // proportional gain for distance (tune)
    private final double kP_ROT = 2.0;    // proportional gain for rotation (tune)
    private final double maxSpeed = 3.0;  // m/s cap (tune to your robot)
    private final double maxRot = 3.0;    // rad/s cap (tune)

    public LimeLightAlign(CommandSwerveDrivetrain drivetrain, String limelightName) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // nothing special
    }

    @Override
    public void execute() {
        // Safe read of pose estimate
        PoseEstimate pe = LimelightHelpers.getBotPoseEstimate(limelightName, "pose", false);

        // If the helper returned null or no tags, stop robot and return
        if (pe == null || pe.tagCount <= 0) {
            // stop the drivetrain
            drivetrain.applyRequest(() -> new LegacySwerveRequest()
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
            );
            return;
        }

        // prefer averaged distance if provided, otherwise fall back to first fiducial's distToRobot
        double currentDist = pe.avgTagDist;
        if ((Double.isNaN(currentDist) || currentDist <= 0.0001) && pe.rawFiducials != null && pe.rawFiducials.length > 0) {
            currentDist = pe.rawFiducials[0].distToRobot;
        }

        // defensive: if still invalid, stop
        if (Double.isNaN(currentDist) || currentDist <= 0.0001) {
            drivetrain.applyRequest(() -> new LegacySwerveRequest()
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
            );
            return;
        }

        // compute distance error (positive => we are too far, need to drive forward)
        double error = currentDist - targetMeters;
        // P controller for forward/back motion
        double forwardSpeed = kP_DIST * error;

        // clamp speed
        forwardSpeed = MathUtil.clamp(forwardSpeed, -maxSpeed, maxSpeed);

        // rotation control: try to level robot to face tag using fiducial tx (preferred)
        double rotCmd = 0.0;
        if (pe.rawFiducials != null && pe.rawFiducials.length > 0) {
            // raw fiducial tx (txnc) is often the x offset from centerline in normalized coords or degrees
            double tx = pe.rawFiducials[0].txnc;
            // If tx looks like degrees (abs <= 180), use it; else assume normalized pixels -> small number
            rotCmd = kP_ROT * Math.toRadians(tx); // convert degrees to radians if tx is degrees
        } else {
            // fallback: no fine rotation info, just keep zero
            rotCmd = 0.0;
        }
        rotCmd = MathUtil.clamp(rotCmd, -maxRot, maxRot);

        // Build and apply request (drive forward/back, no strafe)
        final double vx = forwardSpeed; // forward along robot x
        final double vy = 0.0;          // no strafing
        final double wz = rotCmd;       // rotation

        drivetrain.applyRequest(() -> new LegacySwerveRequest()
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(wz)
        );
    }

    @Override
    public void end(boolean interrupted) {
        // stop robot on finish/interrupt
        drivetrain.applyRequest(() -> new LegacySwerveRequest()
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0)
        );
    }

    @Override
    public boolean isFinished() {
        // Optionally finish when close enough to target
        PoseEstimate pe = LimelightHelpers.getBotPoseEstimate(limelightName, "pose", false);
        if (pe == null || pe.tagCount <= 0) return false;
        double currentDist = pe.avgTagDist;
        if ((Double.isNaN(currentDist) || currentDist <= 0.0001) && pe.rawFiducials != null && pe.rawFiducials.length > 0) {
            currentDist = pe.rawFiducials[0].distToRobot;
        }
        if (Double.isNaN(currentDist)) return false;
        // finish if within 6 inches (0.1524 m) of the desired distance
        return Math.abs(currentDist - targetMeters) < 0.1524;
    }
}
