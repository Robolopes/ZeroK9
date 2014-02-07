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

/**
 * The VM is configured to automatically run this class, and to call the
 * methods corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
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
     * Set to true if using tank drive
     */
    private static final boolean useTankDrive = true;
    
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
    private final DriverStationEnhancedIO driverStation = DriverStation.getInstance().getEnhancedIO();
    
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
     * Initialize compressor
     */
    private final int pressureSwitchChannel = 1;
    private final int compressorRelayChannel = 1;
    private final Compressor compressor = new Compressor(pressureSwitchChannel, compressorRelayChannel);
    
    /*
     * Initialize relay to control 2012 cooling fan
     */
    // TODO: Kept for reference, remove or change for 2013
    private final Relay fan = new Relay(2);

    /*
     * Initialize shooter motor data
     * Numbers are control channels of shooter motors
     */
    private final Talon shooterMotorA = new Talon(5);
    private final Talon shooterMotorB = new Talon(6);
    private final Joystick shooterJoystick = operatorStick;
    private final int shooterMotorLoadButton = 8;
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
    boolean isMovingToMiddle = false;
    DigitalInput clawMiddleSwitch = new DigitalInput(4);

    
    /*
     * Initialize values for autonomous control
     */
    private long startTime = 0;
    
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
        //robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        //robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
    }
    
    /*
     * This method is run when autonomous mode starts
     */
    public void autonomousInit() {
        startTime = System.currentTimeMillis();
        // Start compressor
        compressor.start();
        // Turn on cooling fan at beginning of teleop
        // TODO: Kept for reference, remove or change for 2013
        fan.set(Relay.Value.kForward);
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
        
        /*
         * Set time intervals for autonomous
         * Times are in milliseconds
         */
        final long delayTime = 0;
        final long driveTime = delayTime + 1500;
        final double driveSpeed = 0.6;
        final long motorStartUpTime = 2000;
        final long loaderRetractTime = 1000;
        final double shooterSpeed = 0.8;
        final long shotDuration = 2000;
        final long firstShotTime = driveTime + motorStartUpTime;
        final long secondShotTime = firstShotTime + shotDuration;
        final long thirdShotTime = secondShotTime + shotDuration;
        final long fourthShotTime = thirdShotTime + shotDuration;
        final long finishFourthShotTime = fourthShotTime + shotDuration;
        
        String autoMode = "None";
        if(elapsed > 0 && elapsed < delayTime) {
            autoMode = "Waiting ...";
            /*
             * Wait before starting autonomous logic
             */
            robotDrive.tankDrive(0.0, 0.0);
        } else if(elapsed > delayTime && elapsed < driveTime) {
            autoMode = "Driving forward";
            /*
             * Drive forward for first 2.5 sec of autonomous
             * Start shooter motors so ready to shoot when stop.
             */
            superShifter.set(true);
            isSuperShifterLow = true;
            robotDrive.tankDrive(-driveSpeed, -driveSpeed);
        } else if (elapsed > driveTime && elapsed < firstShotTime) {
            autoMode = "Do something";
        } else {
            autoMode = "Done";
            /*
             * Done moving and shooting, shut down
             */
            robotDrive.arcadeDrive(0.0, 0.0);
            setClawMotors(0.0);
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
        // TODO: Kept for reference, remove or change for 2013
        fan.set(Relay.Value.kForward);
        // Shift to high for teleop
        superShifter.set(false);
        isSuperShifterLow = false;
    }

    /**
     * This method is called periodically during operator control
     */
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Telop Match Time ", m_ds.getMatchTime());
        SmartDashboard.putNumber("Telop Packet Number ", m_ds.getPacketNumber());
        boolean pressure = compressor.getPressureSwitchValue();
        SmartDashboard.putBoolean("Pressure Sensor ", pressure);
        
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
        if(clawJoystick.getRawButton(clawMotorButtonGrab) && !clawGrabSwitch.get()) {
            // Set claw motor to Grab
            setClawMotors(0.3);
        } else if(clawJoystick.getRawButton(clawMotorButtonRelease)) {
            // Set Claw motor to Relase
            setClawMotors(-0.3);
        } else {
            // Turn off shooter motors
            setClawMotors(0.0);
        }
                       /*
         * Set claw arm oontrols
         */
        if(clawJoystick.getRawButton(clawButtonUp)) {
            setClawArms("up");
        } else if(clawJoystick.getRawButton(clawButtonMiddle)) {
             setClawArms ("middle");
        } else if(clawJoystick.getRawButton(clawButtonDown)) {
            setClawArms("down");
        }
        if (isMovingToMiddle && clawMiddleSwitch.get()) {
            stopClawArms();
        }
    }
    
    /*
     * This method sets shooter motors
     * 
     * @param value motor speed
     */
    public void setClawMotors(double value) {
        clawMotorA.set(value);
        clawMotorB.set(value);
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
    
    /*
     * Set shoot solenoid
     * 
     */
    public void setShootSolenoid(boolean shoot) {
        shootSolenoid.set(shoot);
        SmartDashboard.putBoolean("Shoot ", shoot);
    }
    
    /*
     * Set claw to up, middle, or down (false)
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
            } else if (currentClawPosition.equals("down")) {
                clawSolenoidUpA.set(true);
                clawSolenoidUpB.set(true);
                clawSolenoidDownA.set(false);
                clawSolenoidDownB.set(false);
            }
            isMovingToMiddle = true;
            currentClawPosition = "middle";
        } else if (mode.equalsIgnoreCase("down")) {
            clawSolenoidUpA.set(false);
            clawSolenoidUpB.set(false);
            clawSolenoidDownA.set(true);
            clawSolenoidDownB.set(true);
            currentClawPosition = "down";
        }
    }
    
    /*
     * Set lift arms to up (true) or down (false)
     * 
     * @param liftUp true extends arms up, false retracts down.
     */
    public void stopClawArms() {
        clawSolenoidUpA.set(false);
        clawSolenoidUpB.set(false);
        clawSolenoidDownA.set(false);
        clawSolenoidDownB.set(false);
        isMovingToMiddle = false;
    }
    
    /**
     * This method is called periodically during test mode
     */
    public void testPeriodic() {
        long time = System.currentTimeMillis();
        SmartDashboard.putNumber("Test Mode Running Time ", time);
    }
    
}
