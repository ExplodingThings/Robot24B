package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.PhotonVision;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowTarget extends Command {
    PIDController rotationController = new PIDController(0.01, 0, 0); // for rotating drivebase
    PIDController translationController = new PIDController(0.1, 0, 0); // for moving drivebase in X,Y plane
    DriveBase robotDrive;
    PhotonVision photonVision;

    TargetTypes targetType;
    double maxSpeed;
    float distanceFromTarget;
    boolean faceTarget;
    
    public enum TargetTypes { AprilTag, Note }

    /**
     * Drive to a target using functionality based on targetType
     * @param robotDrive the robot drive base
     * @param photonVision the photonvision subsystem
     * @param targetType whether a april tag or note is being followed
     * @param distanceFromTarget camera FOV percentage for note, otherwise magnitude distance
     * @param faceTarget whether the robot rotates to attempt to face the target, IGNORED FOR NOTE
     */
    public FollowTarget(DriveBase robotDrive,
                        PhotonVision photonVision,
                        TargetTypes targetType,
                        float maxSpeed,
                        float distanceFromTarget,
                        boolean faceTarget) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;
        this.targetType = targetType;
        this.maxSpeed = maxSpeed;
        this.distanceFromTarget = distanceFromTarget;
        this.faceTarget = faceTarget;

        addRequirements(robotDrive);
    }


    @Override
    public void initialize() {
        Util.consoleLog();

        // photonVision.selectPipeline(0); // TODO: set pipeline

        rotationController.setSetpoint(0); // target should be at yaw=0 degrees
        rotationController.setTolerance(0.5); // within 0.5 degrees of 0

        translationController.setSetpoint(distanceFromTarget);
    }

    @Override
    public void execute() {
        switch (targetType) {
            case AprilTag:
                moveToAprilTag();
                break;
            case Note:
                moveToNote();
                break;
        }
    }
    private void moveToAprilTag() {
        Transform3d robotToPose = photonVision.getRobotToTag().orElse(null);
        if (robotToPose == null) return;

        double robotToPoseMagnitude = Math.sqrt(Math.pow(robotToPose.getX(), 2)
                                             + Math.pow(robotToPose.getY(), 2));
        
        double speedMultiplier = translationController.calculate(1 / robotToPoseMagnitude * maxSpeed);
        double rot = faceTarget ? rotationController.calculate(photonVision.getYaw()) : 0;

        robotDrive.driveRobotRelative(robotToPose.getX() * speedMultiplier,
                                      robotToPose.getY() * speedMultiplier,
                                      rot);
    }
    private void moveToNote() {
        // make sure target centered before we move
        if (!rotationController.atSetpoint()) {
            double rotation = rotationController.calculate(photonVision.getYaw());
            robotDrive.driveRobotRelative(0, 0, rotation);
        }
        // otherwise drive to the target (only forwards backwards)
        else {
            double movement = translationController.calculate(photonVision.getArea());
            robotDrive.driveRobotRelative(0, -movement, 0); // negative because camera backwards
        }
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        
        robotDrive.driveRobotRelative(0, 0, 0);
    }
}

