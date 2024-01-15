package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Timer; //Unused, but here.

public class LimeLight {
    //setup networktable upon creation
    private NetworkTable nTable = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = nTable.getEntry("tx");
    private NetworkTableEntry ty = nTable.getEntry("ty");
    private NetworkTableEntry ta = nTable.getEntry("ta");
    private NetworkTableEntry tv = nTable.getEntry("tv");

    private double currentX; // X value is horizontal angle from center of LL camera
    private double currentY; // Y value is vertical angle from center of LL camera
    private double currentArea; // Unknown what this does currently.
    private double seesTarget;
    //BOOLEANS
    private boolean targetInView = false;
    //TIMERS
    private final Timer seekTimer = new Timer();
    private final Timer driveTimer = new Timer();

    //CONSTANTS
    //Physical distance of limelight LENS from ground (measured in INCHES)
    private final double LensDistFromGround = 10.75;
    //Physical vertical angle of lens from mount (measured in DEGREES).
    private final double LensAngleFromMount = 93.57;
    //Physical height of chosen AprilTag.
    //If needed, create a table that holds the AprilTag IDs and its height from the ground.
    private final double targetHeight = 32.0;
    //Correction modifier. I assume it designates how much of a correction you want.
    private final double correctionMod = -.1;
    //Preset distance from target.
    //Could put it in an array and designate it to an AprilTag.
    private final double desiredDist = 24.0;

    //#LIMELIGHT
    /* Constructor. Assigns values to the coordinate variables above.
    */
    public LimeLight(){
        //Start catching limelight values
        currentX = tx.getDouble(0.0);
        currentY = ty.getDouble(0.0);
        currentArea = ta.getDouble(0.0);
        seesTarget = tv.getDouble(0.0);
        //Make them visible (via SmartDashboard)
        SmartDashboard.putNumber("LimelightX", this.currentX);
        SmartDashboard.putNumber("LimelightY", this.currentY);
        SmartDashboard.putNumber("LimelightArea", this.currentArea);
        SmartDashboard.putNumber("LimeLightSeesTarget", this.seesTarget);
    }
    //#ESTIMATEDIST
    /* Does math to estimate the distance from the limelight to the target.
        Assumes that robot is centered on target.
     */
    public double estimateDist(){
        double radAngle = Math.toRadians(this.currentY + LensAngleFromMount);

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
    public void getInRange(DriveTrain driveTrain){

        postValues();
        if (driveTimer.get() <= 0.0){
            driveTimer.start();
        } else if (driveTimer.get() > 0.0 && targetInView) {
            double currentDist = estimateDist();
            double distError = desiredDist - currentDist; //Distance from desired point. Calculated in Inches.
    
            if (distError > .5 || distError < -.5){
    
                postValues();
    
                double drivingAdjust  = (correctionMod * distError) * .1; //% of angle (i think)
                double speed = .7;
                if (drivingAdjust > 0)
                    speed = .7;
                else if (drivingAdjust < 0)
                    speed = -.7;
                //Cap turn power at 70% of value
                double turnPower = (this.currentX * 0.7) * drivingAdjust; 
                driveTrain.HamsterDrive.arcadeDrive(speed, turnPower);
    
                distError = desiredDist - this.estimateDist(); //Distance from desired point.
            } else {
                driveTrain.HamsterDrive.arcadeDrive(0, 0);
                driveTimer.stop();
                driveTimer.reset();
            }
        }
    }
    /*#SEEKTARGET
     * Turns the robot until the limelight catches a glimpse of the target
     * On the robot seeing it, centers on the target with a .5 degree range of error.
     * Unknown which way the directions are.
     */
    public void seekTarget(DriveTrain driveTrain){
        double steeringPow = .3;
        if (seekTimer.get() <= 0.0){
            seekTimer.start();
        } else {
            if (seesTarget == 0.0){
                targetInView = false;
                steeringPow = .3;
                driveTrain.HamsterDrive.arcadeDrive(0, steeringPow);
            } else {
                if (this.currentX > .5 || this.currentX < -.5){
                    if (this.currentX > 0)
                        steeringPow = .3;
                    else if (this.currentX < 0)
                        steeringPow = -.3;
                    driveTrain.HamsterDrive.arcadeDrive(0, steeringPow);
                } else {
                    driveTrain.HamsterDrive.arcadeDrive(0, 0);
                    targetInView = true;
                    seekTimer.stop();
                    seekTimer.reset();
                }
            }
        }
    }

    public void postValues(){
        currentX = tx.getDouble(0.0);
        currentY = ty.getDouble(0.0);
        currentArea = ta.getDouble(0.0);
        seesTarget = tv.getDouble(0.0);

        SmartDashboard.putNumber("LimelightX", this.currentX);
        SmartDashboard.putNumber("LimelightY", this.currentY);
        SmartDashboard.putNumber("LimelightArea", this.currentArea);
        SmartDashboard.putNumber("LimeLightSeesTarget", this.seesTarget);
    }
}
