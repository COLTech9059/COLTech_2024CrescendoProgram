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
    static CANSparkMax shooterMotor = new CANSparkMax(Constants.shooterID, MotorType.kBrushless);

    //Create the encoder objects
    static RelativeEncoder leftBaseEncoder = leftBaseMotor.getEncoder();
    static RelativeEncoder rightBaseEncoder = rightBaseMotor.getEncoder();
    static RelativeEncoder ampEncoder = ampMotor.getEncoder();
    static RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

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
        shooterMotor.restoreFactoryDefaults();

        //Set the leftBaseMotor as a follower
        leftBaseMotor.follow(rightBaseMotor);

        //Set the encoders to 0, effectively resetting them
        leftBaseEncoder.setPosition(0);
        rightBaseEncoder.setPosition(0);
        ampEncoder.setPosition(0);
        shooterEncoder.setPosition(0);
    }

    static Timer resetTime = new Timer();
    private static boolean isReset;

    //#RESETENCODERS
    //This method will move the base motors a little bit, nearly guaranteeing accuracy on the encoders
    public static void resetEncoders() {

        //Reset and start the resetTime timer
        resetTime.reset();
        resetTime.start();

        //Move the motors a little bit to increase accuracy for 1 second
        if (resetTime.get() <= 1) {
        isReset = false;
        rightBaseMotor.set(0.3);
        } else if(isReset == false && resetTime.get() > 1) {
            resetTime.stop();
            rightBaseMotor.set(0);
            rightBaseEncoder.setPosition(0);
            leftBaseEncoder.setPosition(0);
            isReset = true;
        }
    }
        //#MANIPULATORDASHBOARD
        //This method updates the dashboard with all the data from the manipulator class
        public static void manipulatorDashboard() {
            SmartDashboard.putBoolean("Beam Sensor", beamSensor.get());
            SmartDashboard.putBoolean("Magnetic Sensor", magneticSensor.get());
        }
}
