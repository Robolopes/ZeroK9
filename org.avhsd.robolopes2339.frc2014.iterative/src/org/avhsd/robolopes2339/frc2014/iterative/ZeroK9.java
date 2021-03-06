/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.avhsd.robolopes2339.frc2014.iterative;

/*
 * Import necessary Java classes
 * Netbeans (or Eclipse) usually manages this automatically
 * This is used to let rest of code know how other classes work.
 */
import edu.wpi.first.wpilibj.AnalogModule;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Dashboard;
import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO.EnhancedIOException;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.PIDController;

/**
 * The VM is configuredLow to automatically run this class, and to call the
 methods corresponding to each mode, as described in the IterativeRobot
 documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class ZeroK9 extends IterativeRobot {
    /*
     * This is the beginning of our main robot class, LeeroyJenkins.
     * When our robot runs, one object of the LeeroyJenkins class will control
     * the robot.
     * 
     * It is common to define variables at the beginning of a class.
     * Variables store data.
     * 
     * After the variables are defined, the rest of the class is used to define 
     * methods. Methods perform actions. They do things for the class
     */
    
    /**********************
     * CLASS VARIABLES
    /**********************/
    
    /*
     * Smart dashboard preferences
     */
    private final Preferences preferences = Preferences.getInstance();
    private int redLow;
    
    /*
     * Initialize joystck variables.
     * The (1) and (2) refer hardward channels 1 and 2 on the robot
     */
    private final Joystick driveStickLeft = new Joystick(1);
    // Only need this if using Tank drive
    private final Joystick driveStickRight = new Joystick(2);
    // Third joystick for shooter and lift control
    private final Joystick operatorStick = new Joystick(3);
    
    // Class to interact with driver station
    //private final DriverStationEnhancedIO driverStation = DriverStation.getInstance().getEnhancedIO();
    private final DriverStation driverStation = DriverStation.getInstance();
    
    /*
     * Initialize motor controllers.
     */
    private final Talon m_frontLeft = new Talon(1);
    private final Talon m_frontRight= new Talon(2);
    private final Talon m_rearLeft  = new Talon(3);
    private final Talon m_rearRight = new Talon(4);
    /*
     * Initialize robot drive to match our controllers
     */
    private final RobotDrive robotDrive = 
            new RobotDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);
    
    /*
     * Initialize super shifter
     */
    private final Solenoid superShifter = new Solenoid(1);
    private final int superShifterButton = 1;
    private final Joystick superShifterJoystick = driveStickLeft;
    private boolean isSuperShifterLow = false;
    private boolean wasSuperShifterButtonJustPushed = false;
    
    /*
    * Drive enocoder
    */
    private final Encoder encoderDrive = new Encoder(5, 8);
    //private final Encoder encodeDriveRight = new Encoder(7, 8);
    
    /*
     * Drive PID controller
     * See https://wpilib.screenstepslive.com/s/3120/m/7912/l/79828-operating-the-robot-with-feedback-from-sensors-pid-control
     * See http://www.wbrobotics.com/javadoc/edu/wpi/first/wpilibj/PIDController.html
    */
    private final PIDController drivePidFrontLeft = new PIDController(0.1, 0.001, 0.0, encoderDrive, m_frontLeft);
    private final PIDController drivePidFrontRight = new PIDController(0.1, 0.001, 0.0, encoderDrive, m_frontRight);
    private final PIDController drivePidRearLeft = new PIDController(0.1, 0.001, 0.0, encoderDrive, m_rearLeft);
    private final PIDController drivePidRearRight = new PIDController(0.1, 0.001, 0.0, encoderDrive, m_rearRight);
    
    /*
     * Initialize compressor
     */
    private final int pressureSwitchChannel = 1;
    private final int compressorRelayChannel = 1;
    private final Compressor compressor = new Compressor(pressureSwitchChannel, compressorRelayChannel);
    
    /*
     * Initialize relay to control cooling fan
     */
    private final Relay fan = new Relay(2);

    /*
     * Initialize shooter motor data
     * Numbers are control channels of shooter motors
     */
    private final Talon shooterMotorA = new Talon(5);
    private final Talon shooterMotorB = new Talon(6);
    private final Joystick shooterJoystick = operatorStick;
    private final int shooterWinchMotorLoadButton = 7;
    DigitalInput shooterStopSwitch = new DigitalInput(2);
    private boolean currentLoaderMode = false;
    private long shootButtonTime = 0;
    
   /*
     * Initialize shooter solenoid
   */
    private final Solenoid shootSolenoid = new Solenoid(2);
    private final Joystick shootJoystick = operatorStick;
    private final int shootButton = 1;
    
    
    /*
     * Initialize claw motor data
     * Numbers are control channels of claw motors
     */
    private final Talon clawMotorA = new Talon(7);
    private final Talon clawMotorB = new Talon(8);
    private final Joystick clawJoystick = operatorStick;
    private final int clawMotorButtonGrab = 3;
    private final int clawMotorButtonRelease = 4;
    DigitalInput clawGrabSwitch = new DigitalInput(3);
    
    /*
     * Initialize claw solenoid data
     */
    private final Solenoid clawSolenoidUpA = new Solenoid(3);
    private final Solenoid clawSolenoidDownA = new Solenoid(4);
    private final Solenoid clawSolenoidUpB = new Solenoid(5);
    private final Solenoid clawSolenoidDownB = new Solenoid(6);
    private final int clawButtonUp = 5;
    private final int clawButtonMiddle = 2;
    private final int clawButtonDown = 6;
    String currentClawPosition = "up";
    DigitalInput clawMiddleSwitch = new DigitalInput(4);
    
    /*
     * Vision class
    */
    private final ZeroK9Vision visionControl = new ZeroK9Vision();

    
    /*
     * Initialize values for autonomous control
     */
    private long startTime = 0;
    private boolean haveShot = false;
    private boolean haveImage = false;
    
    /*
     * Time variables to help with timed printouts
     */
    private long robotStartTime = 0;
    private long time0 = 0;
    
    /**********************
     * CLASS METHODS
     * Methods for this class are below here
    /**********************/
    
    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        robotStartTime = System.currentTimeMillis();
        System.out.println("Robot init time: " + robotStartTime);
        
        redLow = preferences.getInt("Red low", 200);
        //robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        //robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        System.out.println("Before init: " + System.currentTimeMillis());
        visionControl.visionInit();
        System.out.println("After init: " + System.currentTimeMillis());
    }
    
    /*
     * This method is run when autonomous mode starts
     */
    public void autonomousInit() {
        startTime = System.currentTimeMillis();
        System.out.println("Autonomous init time: " + startTime);
        // Start compressor
        compressor.start();
        // Turn on cooling fan at beginning of autonomous
        fan.set(Relay.Value.kForward);
        // Initialize drive encoder
        encoderDrive.reset();
        encoderDrive.setDistancePerPulse(1.0);
        haveShot = false;
        haveImage = false;
    }

    /**
     * This method is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        long elapsed = System.currentTimeMillis() - startTime;
        SmartDashboard.putNumber("Autonomous Elapsed time ", elapsed/1000.0);
        double shooterSlider = 5.0;
        double driveTimeSlider = 2.0;
        double driveSpeedSlider = 5.0;
        SmartDashboard.putNumber("Autonomous Shooter Slider ", shooterSlider);
        SmartDashboard.putNumber("Drive encoder distance", encoderDrive.getDistance());
        SmartDashboard.putBoolean("Drive encoder direction", encoderDrive.getDirection());
        SmartDashboard.putBoolean("Have shot ", haveShot);
        
        /*
         * Set time intervals for autonomous
         * Times are in milliseconds
         */
        final long imageWaitTime = 100;
        final long delayTime = 200;
        // Start bot about 3 ft behind line.
        final long driveTime = delayTime + 1500; 
        final long drivePauseTime = driveTime + 1000;
        final long clawLowerTime = drivePauseTime + 300;
        final double driveSpeed = 1.0;
        final double leftRation = 1.0;
        final long shooterWinchMotorRetractTime = 1900;
        final double shooterWinchMotorSpeed = 1.0;
        
        // Don't use claw motors in autonomous
        setClawMotors(0.0);
        
        // Get image
        if (elapsed > imageWaitTime && !haveImage) {
            haveImage = true;
            System.out.println("Before image: " + System.currentTimeMillis());
            visionControl.getNewTarget(null);
            System.out.println("After image: " + System.currentTimeMillis());
        }
        
        // Retract shooter right away
        if (elapsed < shooterWinchMotorRetractTime && !haveShot) {
            setShootWinchMotors(shooterWinchMotorSpeed); 
        } else {
            setShootWinchMotors(0.0);
        }
        
        String autoMode = "None";
        SmartDashboard.putBoolean("Actve target ", visionControl.isTargetActive());
        if(elapsed < delayTime) {
            autoMode = "Waiting ...";
            /*
             * Wait before starting autonomous logic
             */
            robotDrive.tankDrive(0.0, 0.0);
            stopClawArms();
        } else if(elapsed > delayTime && elapsed < driveTime) {
            autoMode = "Driving forward";
            /*
             * Drive forward for first 2.5 sec of autonomous
             * Start shooter motors so ready to shoot when stop.
             */
            superShifter.set(true);
            isSuperShifterLow = true;
            robotDrive.tankDrive(-leftRation * driveSpeed, -driveSpeed);
            stopClawArms();
        } else if (elapsed > driveTime && elapsed < drivePauseTime) {
            autoMode = "After drive pause";
            robotDrive.tankDrive(0.0, 0.0);
            stopClawArms();
        } else if (elapsed > drivePauseTime && elapsed < clawLowerTime) {
            autoMode = "Lower claw";
            robotDrive.tankDrive(0.0, 0.0);
            // Put claw is down
            setClawArms("down");
        } else if (elapsed > clawLowerTime && elapsed < shooterWinchMotorRetractTime) {
            autoMode = "Retracting";
            robotDrive.tankDrive(0.0, 0.0);
            // Stop claw arm
            stopClawArms();
        } else if (elapsed > clawLowerTime && elapsed > shooterWinchMotorRetractTime && !haveShot) {
            autoMode = "Waiting to shoot";
            robotDrive.tankDrive(0.0, 0.0);
            stopClawArms();
            setShootWinchMotors(0);
            if (visionControl.isTargetActive() || elapsed > 6000) {
                setShootSolenoid(true);
                haveShot = true;
            }
        } else {
            autoMode = "Done";
            stopClawArms();
            /*
             * Done moving and shooting, shut down
             */
            robotDrive.tankDrive(0.0, 0.0);
            if (elapsed > 9000) {
                setShootSolenoid(false);
            }
        }
        SmartDashboard.putString("Autonomous Mode ", autoMode);
    }
    
    /*
     * This method is called at the beginning of operator control
     */
    public void teleopInit() {
        // Start compressor
        compressor.start();
        // Turn on cooling fan at beginning of teleop
        fan.set(Relay.Value.kForward);
        // Shift to high for teleop
        superShifter.set(false);
        isSuperShifterLow = false;
        encoderDrive.reset();
        encoderDrive.setDistancePerPulse(1.0);
    }

    /**
     * This method is called periodically during operator control
     */
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Telop Match Time ", m_ds.getMatchTime());
        SmartDashboard.putNumber("Telop Packet Number ", m_ds.getPacketNumber());
        SmartDashboard.putNumber("Drive encoder distance", encoderDrive.getDistance());
        SmartDashboard.putBoolean("Drive encoder direction", encoderDrive.getDirection());
        
        String operatorButtons = "";
        for (int iiButton = 0; iiButton < 10; iiButton++) {
            if (iiButton > 0) {
                operatorButtons += " ";
            }
            operatorButtons += (iiButton+1) + ":" + operatorStick.getRawButton(iiButton+1);
        }
        SmartDashboard.putString("Operator Buttons ", operatorButtons);
        
        boolean pressure = compressor.getPressureSwitchValue();
        SmartDashboard.putBoolean("Pressure Sensor ", pressure);
        SmartDashboard.putBoolean("Shooter stop switch ", shooterStopSwitch.get());
        SmartDashboard.putBoolean("Claw middle switch ", clawMiddleSwitch.get());
        
        
        /*
         * Keep fan running
         */
        fan.set(Relay.Value.kForward);
        
        /*
         * Get drive data from joystick
         * Tank drive
         */
        double throttleLeft = driveStickLeft.getRawAxis(2);
        double throttleRight = driveStickRight.getRawAxis(2);
        
        /*
         * Print out siginigicant changes in drive info
         */
        SmartDashboard.putNumber("Throttle Left ", throttleLeft);
        SmartDashboard.putNumber("Throttle Right ", throttleRight);
        
        /*
         * Shift with super shifter
         * Shifter button shifts between high and low and back again.
         * Push once, it shifts to high. Push again it shifts to low.
         * The changeSuperShifter method does the shifting.
         * The wasSuperShifterButtonJustPushed flag is used to keep from "bouncing"
         * It makes sure the button is released before asking the shifter to change again.
         * Without it the shifter would shift every 20ms while the button is held down.
         * For example, without the flag, if the button were pushed for 1/2 second
         * the shifter would shift 25 times.
         */
        if(!wasSuperShifterButtonJustPushed && superShifterJoystick.getRawButton(superShifterButton)) {
            changeSuperShifter();
            wasSuperShifterButtonJustPushed = true;
        } else if (wasSuperShifterButtonJustPushed && !superShifterJoystick.getRawButton(superShifterButton)) {
            wasSuperShifterButtonJustPushed = false;
        }
        
        /*
         * Drive robot based on values from joystick
         */
        robotDrive.tankDrive(throttleLeft, throttleRight);
        
        /*
         * Set shooter winch motors
         */
        if(shooterJoystick.getRawButton(shooterWinchMotorLoadButton) && shooterStopSwitch.get()) {
            // Set shooter motors to load
            setShootWinchMotors(1.0);
        } else {
            // Turn off shooter winch motors
            setShootWinchMotors(0.0);
        }
                       /*
        /*
         * Shoot
         */
        if (shootJoystick.getRawButton(shootButton)) {
            setShootSolenoid(true);
        } else {
            setShootSolenoid(false);
        }

        /*
         * Set claw motors
         */
        
        if(clawJoystick.getRawButton(clawMotorButtonGrab) && clawGrabSwitch.get()) {
            // Set claw motor to Grab
            setClawMotors(1.0);
        } else if(clawJoystick.getRawButton(clawMotorButtonRelease)) {
            // Set Claw motor to Relase
            setClawMotors(-1.0);
        } else {
            // Turn off claw motors
            setClawMotors(0.0);
        }
        /*
         * Set claw arm oontrols
         */
        boolean manualStop = true;
        manualStop = driverStation.getDigitalIn(1);
        SmartDashboard.putBoolean("Manual stop", manualStop);
        if (manualStop) {
            if(clawJoystick.getRawButton(clawButtonUp)) {
                setClawArms("up");
            } else if(clawJoystick.getRawButton(clawButtonDown)) {
                setClawArms("down");
            } else {
                stopClawArms();
            }
            
        } else {
            if(clawJoystick.getRawButton(clawButtonUp)) {
                setClawArms("up");
            } else if(clawJoystick.getRawButton(clawButtonMiddle)) {
                setClawArms ("middle");
            } else if(clawJoystick.getRawButton(clawButtonDown)) {
                setClawArms("down");
            }
            SmartDashboard.putBoolean("Claw mag switch", clawMiddleSwitch.get());
            if (!clawMiddleSwitch.get() && currentClawPosition.startsWith("moving")) {
                stopClawArms();
            }
        }
    }
    
    /*
     * Change super shifter from high speed to low speed and back
     * 
     */
    public void changeSuperShifter() {
        superShifter.set(!isSuperShifterLow);
        isSuperShifterLow = !isSuperShifterLow;
        if (isSuperShifterLow) {
            SmartDashboard.putString("Super shifter ", "High");
        } else {
            SmartDashboard.putString("Super shifter ", "Low");
        }
    }
    
    public void pidDriveHold(double holdValue) {
        drivePidFrontLeft.setSetpoint(holdValue);
        drivePidFrontRight.setSetpoint(holdValue);
        drivePidRearLeft.setSetpoint(holdValue);
        drivePidRearRight.setSetpoint(holdValue);
        robotDrive.tankDrive(holdValue, holdValue);
    }
    
    public void pidBrake() {
        superShifter.set(true);
        isSuperShifterLow = true;
        pidDriveHold(0.0);
    }
    
    /*
     * This method sets shooter winch motors
     * 
     * @param value motor speed
     */
    public void setShootWinchMotors(double value) {
        shooterMotorA.set(value);
        shooterMotorB.set(value);
        SmartDashboard.putNumber("Shoot motor value ", value);
    }
    
    /*
     * Set shoot solenoid
     * 
     */
    public void setShootSolenoid(boolean shoot) {
        shootSolenoid.set(shoot);
        SmartDashboard.putBoolean("Shoot ", shoot);
    }
    
    /*
     * This method sets claw motors
     * 
     * @param value motor speed
     */
    public void setClawMotors(double value) {
        clawMotorA.set(-value);
        clawMotorB.set(value);
        SmartDashboard.putNumber("Claw value ", value);
    }
    
    /*
     * Set claw to up, middle, or down
     * 
     * @param liftUp true extends arms up, false retracts down.
     */
    public void setClawArms(String mode) {
        if (mode.equalsIgnoreCase("up")) {
            clawSolenoidUpA.set(true);
            clawSolenoidUpB.set(true);
            clawSolenoidDownA.set(false);
            clawSolenoidDownB.set(false);
            currentClawPosition = "up";
        } else if (mode.equalsIgnoreCase("middle")) {
            if (currentClawPosition.equals("up")) {
                clawSolenoidUpA.set(false);
                clawSolenoidUpB.set(false);
                clawSolenoidDownA.set(true);
                clawSolenoidDownB.set(true);
                currentClawPosition = "moving to middle";
            } else if (currentClawPosition.equals("down")) {
                clawSolenoidUpA.set(true);
                clawSolenoidUpB.set(true);
                clawSolenoidDownA.set(false);
                clawSolenoidDownB.set(false);
                currentClawPosition = "moving to middle";
            }
        } else if (mode.equalsIgnoreCase("down")) {
            clawSolenoidUpA.set(false);
            clawSolenoidUpB.set(false);
            clawSolenoidDownA.set(true);
            clawSolenoidDownB.set(true);
            currentClawPosition = "down";
        }
        SmartDashboard.putString("Claw position ", currentClawPosition);
    }
    
    /*
     * Stop claw arms (used to stop in middle.
     * 
     * @param liftUp true extends arms up, false retracts down.
     */
    public void stopClawArms() {
        clawSolenoidUpA.set(false);
        clawSolenoidUpB.set(false);
        clawSolenoidDownA.set(false);
        clawSolenoidDownB.set(false);
        currentClawPosition = "middle";
        SmartDashboard.putString("Claw position ", currentClawPosition);
    }
    
    /**
     * This method is called at the beginning of test mode
     */
    public void testInit() {
        startTime = System.currentTimeMillis();
        System.out.println("Test init time: " + startTime);
        haveImage = false;
    }
    /**
     * This method is called periodically during test mode
     */
    public void testPeriodic() {
        long time = System.currentTimeMillis();
        SmartDashboard.putNumber("Test Mode Running Time ", time);
        long elapsed = System.currentTimeMillis() - startTime;
        SmartDashboard.putBoolean("Actve target ", visionControl.isTargetActive());

        /*
         * Set wait time before getting image.
         * Time is in milliseconds
         */
        final long imageWaitTime = 100;
        
        // Get image
        if (elapsed > imageWaitTime && !haveImage) {
            haveImage = true;
            System.out.println("Before image: " + System.currentTimeMillis());
            boolean useCameraForImage = true;
            useCameraForImage = driverStation.getDigitalIn(2);
            if (useCameraForImage) {
                /*
                 * Use following line to get image from camera.
                 * This is for on field calibration.
                 */
                visionControl.getNewTarget(null);
            } else  {
                /*
                 * Use following line to get image which is stored on cRIO.
                 * This is for calibrating the code with a stored image.
                 */
                visionControl.getNewTarget("/cameraImage.jpg");
            }
            System.out.println("After image: " + System.currentTimeMillis());
        }
        
    }
    
 }   

