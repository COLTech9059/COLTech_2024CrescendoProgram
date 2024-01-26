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
    private NetworkTableEntry tid = nTable.getEntry("tid");

    private double currentX; // X value is horizontal angle from center of LL camera
    private double currentY; // Y value is vertical angle from center of LL camera
    private double currentArea; // Unknown what this does currently.
    private double seesTarget; //Double value (only 1 or 0) that tells the program if it sees the target.
    private double curTargetID; //Double value designating the current visible AprilTag.

    private double estimDist = 0.0;
    private double showTurnPower = 0.0;
    //BOOLEANS
    private boolean enabled = false;
    private boolean targetFound = false;
    private boolean inPosition = false;
    //TIMERS
    private final Timer seekTimer = new Timer();
    private final Timer driveTimer = new Timer();
    private final Timer refreshTimer = new Timer();
    //HEIGHTS                                     ID 1   ID 2 ID 3 ID 4 ID 5 ID 6    ID 7 
    //ID #s are the number minus 1.
    private final double[] heightArray = {50.125, 53.88, 0.0, 0.0, 0.0, 0.0, 50.125, 53.88};

    //CONSTANTS
    //Physical distance of limelight LENS from ground (measured in INCHES)
    private final double LensDistFromGround = 11.25;
    //Physical vertical angle of lens from mount (measured in DEGREES).
    private final double LensAngleFromMount = 41.9;
    //Physical height of chosen AprilTag.
    //If needed, create a table that holds the AprilTag IDs and its height from the ground.
    // private final double targetHeight = 53.88;
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
        currentX = tx.getDouble(0.0);
        currentY = ty.getDouble(0.0);
        currentArea = ta.getDouble(0.0);
        seesTarget = tv.getDouble(0.0);
        curTargetID = tid.getDouble(0.0);
        //Make them visible (via SmartDashboard)
        SmartDashboard.putNumber("LimelightX", this.currentX);
        SmartDashboard.putNumber("LimelightY", this.currentY);
        SmartDashboard.putNumber("LimelightArea", this.currentArea);
        SmartDashboard.putNumber("LimeLightSeesTarget", this.seesTarget);
        SmartDashboard.putNumber("DistFromTarget", estimDist);
        SmartDashboard.putNumber("VisibleTargetID", this.curTargetID);

        enabled = true;
    }
    //#ESTIMATEDIST
    /* Does math to estimate the distance from the limelight to the target.
        Assumes that robot is centered on target.
     */
    private double targetHeight = 0.0;
    public double estimateDist(){
        int tempID = (int) curTargetID;
        if (tempID > 0.0){
            targetHeight = heightArray[tempID];
        }
        double radAngle = Math.toRadians(this.currentY + LensAngleFromMount);

        //Simple trigonometry to return distance from given angle 
        double distFromGoal = ((targetHeight - LensDistFromGround) / Math.tan(radAngle));
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
        refreshTimer.stop();
        refreshTimer.reset();
        enabled = false;
        targetFound = false;
        inPosition = false;
    }
    //#START
    /*Enables the limelight.
     * Allows for multi-use of autonomous.
     */
    public void start(){
        enabled = true;
        targetFound = false;
        inPosition = false;
    }
    //#GETINRANGEUSINGAREA
    /*An alternative to getInRangeUsingDistance. 
     * Approaches the target using the total area taken rather than a set distance.
     * Used for getting rough approaches.
     */
    public void getInRangeUsingArea(DriveTrain driveTrain){
        if(driveTimer.get() > 3.0 && seesTarget == 0.0){
            driveTrain.HamsterDrive.arcadeDrive(0, 0);
            stop();
        }
         if (enabled){
            if (driveTimer.get() == 0.0 && targetFound) {driveTimer.start(); refreshTimer.start();}
            if (driveTimer.get() > 0.0){
                if (currentArea <= 15.0){
                    double speed = -.35;
                    driveTrain.HamsterDrive.arcadeDrive(speed, 0);
                } else {
                    driveTrain.HamsterDrive.arcadeDrive(0, 0);
                    stop();
                }
            }
        }  
    }
    //#GETINRANGEUSINGDISTANCE
    /* Allows the robot to precisely get in range of a target.
     * THEORETICALLY should work.
     */
    private double speed = .45;
    public void getInRangeUsingDistance(DriveTrain driveTrain){
        if(driveTimer.get() > 3.0 && refreshTimer.get() > 3.0){
            driveTrain.HamsterDrive.arcadeDrive(0, 0);
            stop();
        }
        if (enabled){
            if (driveTimer.get() == 0.0 && targetFound) {driveTimer.start(); refreshTimer.start();}
            if (driveTimer.get() > 0.0){
                if(seesTarget == 1.0){
                    refreshTimer.reset();
                    refreshTimer.start();
                }
                //Estimate distance from target and the distance error.
                double currentDist = estimateDist();
                double distError = desiredDist - currentDist; //Distance from desired point. Calculated in Inches.
                if (distError > 2.5 || distError < -2.5){
                    //Calculate driving adjust percentage for turning.
                    double drivingAdjust  = ((correctionMod * distError) * .1); //% of angle (i think)
                    //Cap the speed at 45% and set the floor at 25%
                    if (drivingAdjust > 0) speed = -.45;
                    else if (drivingAdjust < 0) speed = .45;
                    // if (drivingAdjust < .325 && drivingAdjust > 0.0) speed = .325;
                    // else if (drivingAdjust > -.325 && drivingAdjust < 0.0) speed = -.325;
                    //Cap turn power at 35% of value
                    double turnPower = -Math.pow((this.currentX*.1), 3);
                    if (turnPower < -.35)
                        turnPower = -.35;
                    else if (turnPower > .35)
                        turnPower = .35;
                    showTurnPower = turnPower;
                    driveTrain.HamsterDrive.arcadeDrive(speed, turnPower);
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
           if (seekTimer.get() <= 0.0 && !targetFound) seekTimer.start();
           if (seekTimer.get() > 0.0 && !targetFound){
            //If target isn't in view, set steeringPow to be a consistent .3. 
                if (seesTarget == 0.0){
                    steeringPow = .35;
                    driveTrain.HamsterDrive.arcadeDrive(0, steeringPow);
                } else {
                    //Else if it is visible then...
                    //Runs if it is not in the threshold.
                    if ((currentX > 5.0 || currentX < -5.0) && seesTarget != 0.0){
                        if (currentX > 0.0) steeringPow = -.35;
                        else if (currentX < 0.0) steeringPow = .35;
                        driveTrain.HamsterDrive.arcadeDrive(0, steeringPow);
                    } else if ((currentX < 5 && currentX > -5) && seesTarget != 0.0){
                        //We have found the target. Stop turning.
                        driveTrain.HamsterDrive.arcadeDrive(0, 0);
                        seekTimer.stop();
                        seekTimer.reset();
                        targetFound = true;
                    }
                }
           }
        }
    }
    //#POSTVALUES
    /* Post values from the limelight to variables, then relays them to SmartDashboard for human viewing. 
    */
    public void postValues(){
        //Get values from the limelight.
        currentX = tx.getDouble(0.0);
        currentY = ty.getDouble(0.0);
        currentArea = ta.getDouble(0.0);
        seesTarget = tv.getDouble(0.0);
        curTargetID = tid.getDouble(0.0);
        //Post SmartDashboard values
        SmartDashboard.putNumber("LimelightX", this.currentX);
        SmartDashboard.putNumber("LimelightY", this.currentY);
        SmartDashboard.putNumber("LimelightArea", this.currentArea);
        SmartDashboard.putNumber("LimeLightSeesTarget", this.seesTarget);
        SmartDashboard.putNumber("DistFromTarget", estimDist);
        SmartDashboard.putNumber("VisibleTargetID", this.curTargetID);
        SmartDashboard.putNumber("TurnPowerAdjust", showTurnPower);
    }
}
