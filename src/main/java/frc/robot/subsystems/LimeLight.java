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
    private final NetworkTableEntry tv = nTable.getEntry("tv");

    private double currentX; // X value is horizontal angle from center of LL camera
    private double currentY; // Y value is vertical angle from center of LL camera
    private double currentArea; // Unknown what this does currently.
    private double seesTarget;

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

    //#LIMELIGHT
    /* Constructor. Assigns values to the coordinate variables above.
    */
    public LimeLight(){
        //Start catching limelight values
        this.currentX = tx.getDouble(0.0);
        this.currentY = ty.getDouble(0.0);
        this.currentArea = ta.getDouble(0.0);
        this.seesTarget = tv.getDouble(0.0);
        //Make them visible (via SmartDashboard)
        SmartDashboard.putNumber("LimelightX", currentX);
        SmartDashboard.putNumber("LimelightY", currentY);
        SmartDashboard.putNumber("LimelightArea", currentArea);
        SmartDashboard.putNumber("LimeLightSeesTarget", seesTarget);
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
     * THEORETICALLY should work. At times turnPower is greater than 1 (See simulation), which may cause problems.
     */
    /*TESTING VALUES:
        currentX = 11

        desiredDist = 36 (inches)
        currentDist = 70 (inches)
        distError = 36 - 70 (-34)
        drivingAdjust = (-.1 * -34) (3.4) * .1 = .34
        turnPower = (11 * .7) = ((7.7) * .34) = 2.618

    */
    public boolean getInRange(DriveTrain driveTrain){
        double currentDist = estimateDist();
        double distError = desiredDist - currentDist; //Distance from desired point. Calculated in Inches.

        while (distError > .5 || distError < -.5){
            double drivingAdjust  = (correctionMod * distError) * .1; //% of angle (i think)
            double speed = .7;
            if (drivingAdjust > 0)
                speed = .7;
            else if (drivingAdjust < 0)
                speed = -.7;
            //Cap turn power at 70% of value
            double turnPower = (currentX * 0.7) * drivingAdjust; 
            driveTrain.HamsterDrive.arcadeDrive(speed, turnPower);

            distError = desiredDist - estimateDist(); //Distance from desired point.
        }

        return true;
    }
    /*#SEEKTARGET
     * Turns the robot until the limelight catches a glimpse of the target
     * On the robot seeing it, centers on the target with a .5 degree range of error.
     * Unknown which way the directions are.
     */
    public boolean seekTarget(DriveTrain driveTrain){
        double steeringPow = .3;
        while (seesTarget == 0.0){
            steeringPow = .3;
            driveTrain.HamsterDrive.arcadeDrive(0, steeringPow);
        }
        while (currentX > .5 || currentX < -.5){
            if (currentX > 0)
                steeringPow = .3;
            else if (currentX < 0)
                steeringPow = -.3;
            driveTrain.HamsterDrive.arcadeDrive(0, steeringPow);
        }
        driveTrain.HamsterDrive.arcadeDrive(0, 0);
        return true;
    }

}
