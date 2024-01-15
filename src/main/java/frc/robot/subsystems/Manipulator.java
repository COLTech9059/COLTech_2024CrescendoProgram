package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IO;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Manipulator {
    
    //Create the motor controller objects
    static CANSparkMax leftBaseMotor = new CANSparkMax(Constants.leftBaseID, MotorType.kBrushed);
    static CANSparkMax rightBaseMotor = new CANSparkMax(Constants.rightBaseID, MotorType.kBrushed);
    static CANSparkMax ampMotor = new CANSparkMax(Constants.ampID, MotorType.kBrushless);
    static CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeID, MotorType.kBrushless);

    //Create the encoder objects
    static RelativeEncoder leftBaseEncoder = leftBaseMotor.getEncoder();
    static RelativeEncoder rightBaseEncoder = rightBaseMotor.getEncoder();
    static RelativeEncoder ampEncoder = ampMotor.getEncoder();
    static RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    //Create the digital input objects
    static DigitalInput beamSensor = new DigitalInput(Constants.beamSensorID);
    static DigitalInput magneticSensor = new DigitalInput(Constants.magneticSensorID);

    //#INITIALIZEMANIPULATOR
    //This method will set up the manipulator for use
    public static void initializeManipulator() {

        //Reset the motors to their factory defaults
        leftBaseMotor.restoreFactoryDefaults();
        rightBaseMotor.restoreFactoryDefaults();
        ampMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();

        //Set the leftBaseMotor as a follower
        leftBaseMotor.follow(rightBaseMotor);
        leftBaseMotor.setInverted(true);

        //Set the encoders to 0, effectively resetting them
        leftBaseEncoder.setPosition(0);
        rightBaseEncoder.setPosition(0);
        ampEncoder.setPosition(0);
        intakeEncoder.setPosition(0);
    }



    //#AMPPOSITION
    //This method will run the manipulator base motors until the magnetic sensor is triggered at the amp spitting position
    public static void ampPosition() {

        if (!magneticSensor.get()) {
            rightBaseMotor.set(-0.3);
        } else if (magneticSensor.get()) {
            rightBaseMotor.set(0);
            rightBaseEncoder.setPosition(0);
            leftBaseEncoder.setPosition(0);
        }

    }



        //#MANIPULATORDASHBOARD
        //This method updates the dashboard with all the data from the manipulator class
        public static void manipulatorDashboard() {
            //Push the digital sensor data to the shuffleboard
            SmartDashboard.putBoolean("Beam Sensor", beamSensor.get());
            SmartDashboard.putBoolean("Magnetic Sensor", magneticSensor.get());
        }



        //#INTAKE
        //This method will intake a note
        public static void intake() {
            if (!beamSensor.get()) {
                intakeMotor.set(0.4);
            } else {
                intakeMotor.set(0);
            }
        }



        private static Timer shootTime = new Timer();

        //#SHOOTNOTE
        //This method will shoot a note
        public static void shootNote() {

            shootTime.reset();

            //If the beam sensor is active, the intake motor runs in reverse until the beam sensor is deactivated,
            // at which point the intake motor will stop and the amp motor will run for 1 second at full power to shoot
            if (beamSensor.get()) {
                intakeMotor.set(-0.4);
            } else if (!beamSensor.get()) {
                shootTime.start();
                intakeMotor.set(0);
                ampMotor.set(1);
            } 
            if (!beamSensor.get() && shootTime.get() >= 1) {
                ampMotor.set(0);
                shootTime.stop();
            }
        }


        private static Timer ampTime = new Timer();

        //#AMPSCORE
        //This method will score a note in the amp
        public static void ampScore() {

            ampTime.reset();

            //If the beam sensor is active, the intake motor runs in reverse until the beam sensor is deactivated,
            // at which point the intake motor will stop and the amp motor will run for 1.5 seconds at 40% power to score
            if (beamSensor.get()) {
                intakeMotor.set(-0.4);
            } else if (!beamSensor.get()) {
                ampTime.start();
                intakeMotor.set(0);
                ampMotor.set(0.3);
            } 
            if (!beamSensor.get() && shootTime.get() >= 1.5) {
                ampMotor.set(0);
                ampTime.stop();
            }
        }



        //#MOVEMANIPULATOR
        //This method will move the manipulator forward
        public static void moveManipulator() {
            if (IO.dController.getRightTriggerAxis() > 0.4) {
                rightBaseMotor.set(0.3);
            } else {
                rightBaseMotor.set(0);
            }
        }

        //#MOVEMANIPULATOR
        //This method will move the manipulator forward by a set time
        public static void moveManipulator(double moveTime) {
            
            Timer moveTimer = new Timer();
            if (moveTimer.get() <= moveTime) {
                rightBaseMotor.set(0.3);
            } else {
                rightBaseMotor.set(0);
            }
        }

        //#CONTROLMANIPULATOR
        //This method will add keybinds for all the control methods in the manipulator class
        public static void controlManipulator() {

            if (IO.dController.getXButton()) intake();
            if (IO.dController.getRightTriggerAxis() > 0.4) moveManipulator();
            if (IO.dController.getYButton()) ampPosition();
            if (IO.dController.getRightBumper()) ampScore();
            if (IO.dController.getLeftBumper()) shootNote();

        }



        //#AUTOMANIPULATOR
        //This method will do all of the actions for our manipulator during auto
        public static void autoManipulator(boolean doesIntake, boolean doesAim, boolean doesShoot, boolean doesAmp) {
            if (doesIntake) {
                moveManipulator(1.5);
                intake();
            }
            if (doesAim) ampPosition();
            if (doesShoot) shootNote();
            if (doesAmp) ampScore();
        }
}
