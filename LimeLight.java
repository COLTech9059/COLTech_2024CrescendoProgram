package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    //setup networktable upon creation
    private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry tx = nTable.getEntry("tx");
    private final NetworkTableEntry ty = nTable.getEntry("ty");
    private final NetworkTableEntry ta = nTable.getEntry("ta");

    private double currentX; // X value is horizontal angle from center of LL camera
    private double currentY; // Y value is vertical angle from center of LL camera
    private double currentArea; // Unknown what this does currently.

    //Physical distance of limelight LENS from ground (measured in INCHES)
    private final double LensDistFromGround = 0.0;
    //Physical vertical angle of lens from mount (measured in DEGREES).
    private final double LensAngleFromMount = 0.0;
    //Physical height of chosen AprilTag.
    //If needed, create a table that holds the AprilTag IDs and its height from the ground.
    private final double targetHeight = 0.0;

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

    public double getDistFromTarget(){
        double radAngle = Math.toRadians(currentY + LensAngleFromMount);

        //Simple trigonometry to return distance from given angle (inverse tan function)
        double distFromGoal = (targetHeight - LensDistFromGround) / Math.tan(radAngle);
        return distFromGoal;
    }
    public static void main(String[] args){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }
}
