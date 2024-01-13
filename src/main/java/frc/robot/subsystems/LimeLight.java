package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//Java Imports
import java.util.Timer; //Unused, but here.

public class LimeLight {
    //setup networktable upon creation
    private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry tx = nTable.getEntry("tx");
    private final NetworkTableEntry ty = nTable.getEntry("ty");
    private final NetworkTableEntry ta = nTable.getEntry("ta");

    private double currentX; // X value is horizontal angle from center of LL camera
    private double currentY; // Y value is vertical angle from center of LL camera
    private double currentArea; // Unknown what this does currently.

    //CONSTANTS
    //Physical distance of limelight LENS from ground (measured in INCHES)
    private final double LensDistFromGround = 0.0;
    //Physical vertical angle of lens from mount (measured in DEGREES).
    private final double LensAngleFromMount = 0.0;
    //Physical height of chosen AprilTag.
    //If needed, create a table that holds the AprilTag IDs and its height from the ground.
    private final double targetHeight = 0.0;
    //Correction modifier. I assume it designates how much of a correction you want.
    private final double correctionMod = -.1;
    //Preset distance from target.
    //Could put it in an array and designate it to an AprilTag.
    private final double desiredDist = 36.0;

    //# LIMELIGHT
    /* Constructor. Assigns values to the coordinate variables above.
    */
    public LimeLight(){
        //Start catching limelight values
        this.currentX = tx.getDouble(0.0);
        this.currentY = ty.getDouble(0.0);
        this.currentArea = ta.getDouble(0.0);
        //Make them visible (via SmartDashboard)
        SmartDashboard.putNumber("LimelightX", currentX);
        SmartDashboard.putNumber("LimelightY", currentY);
        SmartDashboard.putNumber("LimelightArea", currentArea);
    }
    //#ESTIMATEDIST
    /* Does math to estimate the distance from the limelight to the target.
        Assumes that robot is centered on target.
     */
    public double estimateDist(){
        double radAngle = Math.toRadians(currentY + LensAngleFromMount);

        //Simple trigonometry to return distance from given angle 
        double distFromGoal = (targetHeight - LensDistFromGround) / Math.tan(radAngle);
        return distFromGoal;
    }
    //#GETINRANGE
    /* Auto-Function that allows the robot to get in range of a given target (currently target ID 1).
     * Not fully optimized currently, slows as you get closer.
     */
    public boolean getInRange(DriveTrain driveTrain){
        double currentDist = estimateDist();
        double distError = desiredDist - currentDist; //Distance from desired point.

        while (distError > .5 || distError < -.5){
            double drivingAdjust  = (correctionMod * distError) * .8; //Basically designates speed.
            if (drivingAdjust < -.5)
                drivingAdjust = -.5;
            else if (drivingAdjust < .5)
                drivingAdjust = .5;
            double turnPower = currentX / 10.0;
            driveTrain.HamsterDrive.arcadeDrive(drivingAdjust, turnPower);
        }

        return true;
    }

}
