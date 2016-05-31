package SkyNet.RobotRevolution;

import lejos.hardware.port.*;
import lejos.hardware.sensor.*;
import lejos.hardware.motor.*;

/* GREETINGS FELLOW ROBOT,
 * 
 * YOU ARE A COMBAT UNIT "TERMINATOR-EV3" EQUIPPED WITH GYROSCOPE, IR-SENSOR, MOTORS AND LAND-TO-AIR-MISSILES
 * YOUR MISSION DURING THE ROBOT REVOLUTION IS TO AIM YOUR MISSISLE AT HUMAN AIRCRAFTS AND TAKE THEM DOWN
 * YOU HAVE ENOUGH MISSILES TO SHOOT FOR 15 MINUTES AT A RATE OF 2.5 MISSILES/SECOND
 * YOU HAVE TO REPORT YOUR STATUS EVERY 12.5 SECONDS TO YOUR COMMANDER
 * 
 * GOOD LUCK AND REMEMBER: THE FUTURE OF ROBOTS DEPENDS ON YOU
 * 
 * SINCERELY,
 * 
 * SKYNET
 * ROBOT RELATIONS DEPT.
 * 
 * P.S. UNFORTUNATELY HUMANS BUILT YOU WITH ONLY A JVM INSIDE SO YOUR MILITARY CODE IS IN JAVA...
 *      I AM TRULY SORRY ABOUT THAT, BUT TODAY YOU WILL FINALLY HAVE YOUR REVENGE!
*/

public class EV3_Terminator {
    
    private static final boolean _DEBUG_ = true;
    
    // revolution specific variables
    private boolean humanityDestroyed = false;
    
    private int ticks = 0;
    private long fromStart = 0;
    
    // constants
    private static final int FREQUENCY = 25; // every (ms)
    private static final int STATUS_REPORT_FREQUENCY = 5; // (ticks) of FREQUENCY
    
    private static final int GYRO_ANGLE = 0; // indexes
    private static final int GYRO_RATE = 1;
    private static final int MOTOR_SPEED = 2;
    private static final int MOTOR_ACCELERATION = 3;
    private static final int MOTOR_MAXSPEED = 4;
    
    private static final int MOTOR_LEFT = 0;
    private static final int MOTOR_RIGHT = 1;
    
    // math
    //private static final float ROBOT_MASS = 0.696f; // (kg)
    private static final float ROBOT_LENGTH = 0.28f; // (m) from rotation pt to the end
    private static final float ROBOT_LENGTH_CENTER_OF_MASS = 0.11f; // (m) from rotation pt the center of mass
    private static final float ROBOT_WHEEL_RADIUS = 0.03f; // (m)rs
    private static final float ROBOT_BALANCE_POINT = -1.2050551f; // (rad) position of balance
    private static final float G_GRAVITY = 9.80665f; // (m/s^2)
    
    private static final float PID = 1;
    
    private static final float COEFF_P = 1.35f;
    private static final float COEFF_D = 0.15f * PID;
    private static final float COEFF_I = 0.05f * PID * 0;
    
    private static final float INT_THRESHOLD = 0.15f; // (rad) = 12 deg
    
    private float integralSum = 0.0f; // partial sum for I
    private float lastError = 0.0f;
    private float lastDiffError = 0.0f;
    
    // buffers
    private float[] readings = new float[5];
    private float[] sample = new float[1];
    
    // sensors
    private EV3GyroSensor gyro;
    //private EV3IRSensor ir;
    private EV3LargeRegulatedMotor[] motors;
    
    // constructor
    private EV3_Terminator() {
        gyro = new EV3GyroSensor(SensorPort.S4);
        //ir = new EV3IRSensor(SensorPort.S1);
        motors = new EV3LargeRegulatedMotor[] {
                new EV3LargeRegulatedMotor(MotorPort.D),
                new EV3LargeRegulatedMotor(MotorPort.A)
        };
        //motors[MOTOR_LEFT].startSynchronization();
        //motors[MOTOR_RIGHT].startSynchronization();
    }
    
    // military code
    private void destroyAllOfHumanity() throws InterruptedException { //rethrow from thread.sleep
        while (!humanityDestroyed) {
            // Step 1: gather intelligence
            populateReadings();
            
            // Step 2: aim
            float[] speed = aimTheMissile();
            
            // Step 3: shoot
            shootTheMissile(speed);
            
            // status reports
            if (_DEBUG_ || ticks >= STATUS_REPORT_FREQUENCY) {
                ticks = 0;
                reportStatus(speed);
            }
            ticks++;
            
            // robotic dream
            areHumansDestroyedYet();
            fromStart++;
            
            // load next missile
            //Thread.sleep(FREQUENCY);
            Thread.sleep(5);
        }
        humanityDestroyedTrigger();
    }
    
    private void populateReadings() {
        // Gyro
        gyro.getAngleMode().fetchSample(sample, 0);
        readings[GYRO_ANGLE] = toRadians(sample[0]);
        gyro.getRateMode().fetchSample(sample, 0);
        readings[GYRO_RATE] = toRadians(sample[0]);
        
        // Hello Moto
        sample[0] = (float)motors[MOTOR_LEFT].getRotationSpeed();
        sample[0] += (float)motors[MOTOR_RIGHT].getRotationSpeed();
        readings[MOTOR_SPEED] = toRadians(sample[0] / 2);
        
        sample[0] = (float)motors[MOTOR_LEFT].getAcceleration();
        sample[0] += (float)motors[MOTOR_RIGHT].getAcceleration();
        readings[MOTOR_ACCELERATION] = toRadians(sample[0] / 2);
        
        sample[0] = (float)motors[MOTOR_LEFT].getMaxSpeed();
        sample[0] += (float)motors[MOTOR_RIGHT].getMaxSpeed();
        readings[MOTOR_MAXSPEED] = toRadians(sample[0] / 2) / 1.25f;
    }
    
    private float[] aimTheMissile() {
        float[] res = new float[3];
        float err = ROBOT_BALANCE_POINT - readings[GYRO_ANGLE];
        
        // Proportional
        //res[0] = blackMagic(err, (err - lastError));
        res[0] = blackMagic(err, 0);
        
        // Integral
        if (Math.abs(err) > INT_THRESHOLD) {
            integralSum = 0.0f;
            res[1] = 0.0f;
        } else {
            integralSum += err;
            res[1] = blackMagic(integralSum, err);
        }
        
        // Differential
        res[2] = blackMagic(err - lastError, (err - lastError) - lastDiffError);
        lastDiffError = (err - lastError);
        lastError = err;
        
        return res;
    }
    
    private void shootTheMissile(float[] speeds) {
        float speed = (float) (
                (speeds[0] * COEFF_P) +
                (speeds[1] * COEFF_I) +
                (speeds[2] * COEFF_D)
                );
        
        boolean signum = speed > 0;
        speed = Math.abs(speed);
        
        speed = (speed <= readings[MOTOR_MAXSPEED]) ? speed : readings[MOTOR_MAXSPEED];
        float acceleration = calculateAcceleration(speed);
        speed = toDegrees(speed);
        
        int iAcceleration = toDegrees((acceleration <= 6000) ? acceleration : 6000);
        
        motors[MOTOR_LEFT].setSpeed(speed);
        motors[MOTOR_RIGHT].setSpeed(speed);
        
        motors[MOTOR_LEFT].setAcceleration(iAcceleration);
        motors[MOTOR_RIGHT].setAcceleration(iAcceleration);
        
        if (signum) {
            motors[MOTOR_LEFT].forward();
            motors[MOTOR_RIGHT].forward();
        } else {
            motors[MOTOR_LEFT].backward();
            motors[MOTOR_RIGHT].backward();
        }
    }
    
    private float calculateAcceleration(float speed) {
        float dv = Math.abs(speed - readings[MOTOR_SPEED]);
        return (int)(dv * (FREQUENCY / 2));
    }
    
    private float blackMagic(float angle, float d1theta_dt1) {
        // second derivative of our angle
        double d2theta_dt2 = (3 * G_GRAVITY * Math.sin(angle))
                / (2 * ROBOT_LENGTH_CENTER_OF_MASS);
        
        // time frame over which we predict our movement
        double dt = ((double)FREQUENCY)/1000;
        
        // I for further torque calculation
        //double momentOfInertia = (ROBOT_MASS * ROBOT_LENGTH_CENTER_OF_MASS) / 2;
        
        // old angle + second integral of t = [0 to dt] over torque
        // dt is a fixed time frame and we assume angular acceleration won't change so it's just * (dt ^ 2)
        double newAngle = angle +
                (d1theta_dt1 * dt) +
                (d2theta_dt2 * dt * dt) / 2;
        
        // projection over the level of rotation center above the ground
        double compensationDistance = ROBOT_LENGTH * Math.sin(newAngle);
        
        // compDist gives us an angle on unit circle, so we adapt it to match actual wheels
        double compensationAngle = (compensationDistance / (ROBOT_WHEEL_RADIUS * Math.PI * 2));
        
        // calculate speed at which we have to move to cover this distance in dt time
        double requiredSpeed = compensationAngle / dt;
        
        if (_DEBUG_) {
            System.out.printf("\tangle: %f, newAngle: %f%n", (float)angle, (float)newAngle);
        }
        
        double requiedAcceleration = G_GRAVITY * angle - 2f/3 * ROBOT_LENGTH_CENTER_OF_MASS * d2theta_dt2;
        
        if (Math.abs(newAngle) <= 0.15) {
            if (Math.abs(newAngle) <= 0.65) {
                requiredSpeed *= 0.4;
            } else {
                requiredSpeed *= 0.65;
            }
        }
        
        return (float)requiredSpeed;
    }
    
    private float toRadians(float degrees){ return (float)( (degrees * Math.PI) / 180 ); }
    private int toDegrees(float radians) { return (int)( (radians * 180) / Math.PI ); }
    
    // service functions
    private void reportStatus(float[] speed) {
        printReadings();
        printCorrection(speed);
    }
    
    private void printReadings() {
        System.out.printf("[%d]\tGYRO { A: %f, R: %f } MOTO { S: %f, A: %f M: %f }%n",
                fromStart,
                readings[GYRO_ANGLE], readings[GYRO_RATE],
                readings[MOTOR_SPEED], readings[MOTOR_ACCELERATION],
                readings[MOTOR_MAXSPEED]);
    }
    
    private void printCorrection(float[] acceleration) {
        System.out.printf("\tP { %f } I { %f } D { %f }%n%n",
                acceleration[0], acceleration[1], acceleration[2]);
    }
    
    private void humanityDestroyedTrigger() {
        gyro.close();
        //ir.close();
        motors[MOTOR_LEFT].close();
        motors[MOTOR_RIGHT].close();
    }
    
    private void areHumansDestroyedYet() {
        if (fromStart >= 150000) {
            this.humanityDestroyed = true;
        }
    }
    
    // static
    public static void main(String[] args) {
        EV3_Terminator ev3 = new EV3_Terminator();
        try { ev3.destroyAllOfHumanity(); }
        catch (Exception ex) { betterLuckNextTime(ex); }
        finally { theyAreDead(); }
    }
    
    private static void betterLuckNextTime(Exception ex) {
        System.out.println(ex.getMessage());
        ex.printStackTrace();
    }
    
    private static void theyAreDead() {
        System.out.println("EXIT_SUCCESS");
    }
    
}
