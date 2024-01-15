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
    private double seesTarget; //Double value (only 1 or 0) that tells the program if it sees the target.

    private double estimDist = 0.0;
    //BOOLEANS
    private boolean enabled = false;
    private boolean targetFound = false;
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
    private final double correctionMod = -1;
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
        SmartDashboard.putNumber("DistFromTarget", estimDist);

        enabled = true;
    }
    //#ESTIMATEDIST
    /* Does math to estimate the distance from the limelight to the target.
        Assumes that robot is centered on target.
     */
    public double estimateDist(){
        double radAngle = Math.toRadians(this.currentY + LensAngleFromMount);

        //Simple trigonometry to return distance from given angle 
        double distFromGoal = (targetHeight - LensDistFromGround) / Math.tan(radAngle);
        estimDist = distFromGoal;
        return distFromGoal;
    }
    //#STOP
    /* Force-Stops all limelight functionality.
     * Used whenever the time-out limit is reached to prevent penalties.
    */
    public void stop(){
        seekTimer.stop();
        seekTimer.reset();
        driveTimer.stop();
        driveTimer.reset();
        enabled = false;
        targetFound = false;
    }
    //#START
    /*Enables the limelight.
     * Allows for multi-use of autonomous.
     */
    public void start(){
        enabled = true;
    }
    //#GETINRANGE
    /* Auto-Function that allows the robot to get in range of a given target (currently target ID 1).
     * THEORETICALLY should work.
     */
    /*TESTING VALUES:
        currentX = 11

        desiredDist = 24 (inches)
        currentDist = 70 (inches)
        distError = 24 - 70 (-46)
        drivingAdjust = (-1 * -34) (-34) * .01 = .034
        turnPower = (5 * .7) = ((3.5) * .034) = .119

    */
    public void getInRange(DriveTrain driveTrain){
        if(driveTimer.get() > 3.0 && seesTarget == 0.0){
            driveTrain.HamsterDrive.arcadeDrive(0, 0);
            stop();
        }
        if (enabled){
            // postValues();
            if (driveTimer.get() == 0.0 && targetFound) driveTimer.start();
            if (driveTimer.get() > 0.0){
                //Estimate distance from target and the distance error.
                double currentDist = estimateDist();
                double distError = desiredDist - currentDist; //Distance from desired point. Calculated in Inches.
                if (distError > 1 || distError < -1){
                    //Calculate driving adjust percentage for turning.
                    double drivingAdjust  = (correctionMod * distError) * .001; //% of angle (i think)
                    double speed = .5;
                    if (drivingAdjust > 0)
                        speed = .5;
                    else if (drivingAdjust < 0)
                        speed = -.5;
                    //Cap turn power at 70% of value
                    double turnPower = (this.currentX * 0.7) * drivingAdjust; 
                    if (turnPower > .7)
                        turnPower = .7;
                    else if (turnPower < -.7)
                        turnPower = -.7;
                    driveTrain.HamsterDrive.arcadeDrive(speed, turnPower);
                    //Recalculate.
                    distError = desiredDist - this.estimateDist(); //Distance from desired point.
                } else {
                    driveTrain.HamsterDrive.arcadeDrive(0, 0);
                    stop();
                }
            }
        }  
    }
    /*#SEEKTARGET
     * Turns the robot until the limelight catches a glimpse of the target
     * On the robot seeing it, centers on the target with a .5 degree range of error.
     * Unknown which way the directions are.
     */
    private double steeringPow = .3;
    public void seekTarget(DriveTrain driveTrain){
        if (seekTimer.get() > 10.0 && seesTarget == 0.0){
            driveTrain.HamsterDrive.arcadeDrive(0, 0);
            stop();
        }
        if (enabled) {
           if (seekTimer.get() <= 0.0) seekTimer.start();
           if (seekTimer.get() > 0.0){
            //If target isn't in view, set steeringPow to be a consistent .3. 
                if (seesTarget == 0.0){
                    steeringPow = .3;
                    driveTrain.HamsterDrive.arcadeDrive(0, steeringPow);
                } else {
                    //Else if it is visible then...
                    //Runs if it is not in the threshold.
                    if (currentX > .5 || currentX < -.5){
                        if (currentX > 0) steeringPow = -.3;
                        else if (currentX < 0) steeringPow = .3;
                        driveTrain.HamsterDrive.arcadeDrive(0, steeringPow);
                    } else {
                        //We have found the target. Stop turning.
                        driveTrain.HamsterDrive.arcadeDrive(0, 0);
                        seekTimer.stop();
                        targetFound = true;
                    }
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
        SmartDashboard.putNumber("DistFromTarget", estimDist);
    }
}
