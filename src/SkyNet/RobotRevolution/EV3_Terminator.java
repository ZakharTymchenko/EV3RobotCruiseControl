package SkyNet.RobotRevolution;

import lejos.hardware.port.*;
import lejos.hardware.sensor.*;
import lejos.hardware.motor.*;

/* Short documentation
 * 1)  Robot is meant to start up in a horizontal position (on his back wheels)
 *     While this is probably inconvenient this was meant to give more consistent gyro readings. Because gyro 
 *     is always initialized in a way that 0 angle and 0 rate represent the position of robot during the
 *     sensor startup. ROBOT_BALANCE_POINT is an angle of vertical position assuming this sort of start.
 * 
 * 2)  Solution involves some hacks and magical numbers because it wouldn't even remotely work otherwise.
 *     Diff reaction is just plain wrong. We calculate angle rate on the runtime over the ~5ms span.
 *     Logically it shouldn't be huge because FREQUENCY constant that defines dt is higher. But it still
 *     manages to have unreasonable overshooting entirely off the diff part even with ~10% coeff.
 *     But at the same time lowering it makes robot who already has slow reactions to most things even slower.
 *     The reason is likely in our physics calculations but none of us is proficient enough with it to be sure.
*/

public class EV3_Terminator {
    private static final float PID_ON = 1f; // 1f (PID-control) or 0f (P-only control)
    
    // global variable for shutdown
    private boolean endOfLifeSpan = false;
    
    private int ticks = 0;
    private long fromStart = 0;
    
    // constants
    private static final int FREQUENCY = 30; // (ms) basis for our time
    private static final int SLEEP_FREQ = 5; // (ms) should be a divisor of FREQ.
    private static final int STATUS_REPORT_FREQUENCY = 10; // (ticks) of FREQUENCY
    
    // indexes
    private static final int GYRO_ANGLE = 0; 
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
    private static final float ROBOT_WHEEL_RADIUS = 0.03f; // (m)
    private static final float ROBOT_BALANCE_POINT = -1.2087f; // (rad) position of balance
    private static final float G_GRAVITY = 9.80665f; // (m/s^2)
    
    private static final float COEFF_P = 0.825f;
    private static final float COEFF_D = 0.105f * PID_ON;
    private static final float COEFF_I = 0.050f * PID_ON;
    
    private static final float INT_THRESHOLD = 0.11f; // (rad)
    
    private float integralSum = 0.0f; // partial sum for I
    private float lastError = 0.0f; // last error to track D
    
    // buffers
    private float[] readings = new float[5];
    private float[] sample = new float[1];
    
    // sensors
    private EV3GyroSensor gyro;
    private EV3LargeRegulatedMotor[] motors;
    
    // constructor
    private EV3_Terminator() {
        gyro = new EV3GyroSensor(SensorPort.S4);
        motors = new EV3LargeRegulatedMotor[] {
                new EV3LargeRegulatedMotor(MotorPort.D),
                new EV3LargeRegulatedMotor(MotorPort.A)
        };
    }
    
    // main code
    private void balancingLoop() throws InterruptedException { //rethrow from thread.sleep
        while (!endOfLifeSpan) {
            // Step 1: gather data
            populateReadings();
            
            // Step 2: calculate
            float[] speed = calculatePID();
            
            // Step 3: execute
            updateMotors(speed);
            
            // status reports
            if (ticks >= STATUS_REPORT_FREQUENCY) {
                ticks = 0;
                reportStatus(speed);
            } else {
                ticks++;
            }
            
            // check if we have to stop
            checkLifespan();
            fromStart++;
            
            // wait
            Thread.sleep(SLEEP_FREQ);
        }
        lifespanFinishedTrigger();
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
    
    private float[] calculatePID() {
        float[] res = new float[3];
        float err = ROBOT_BALANCE_POINT - readings[GYRO_ANGLE];
        
        // Proportional
        res[0] = physicsOfPID(err, 0, PID.P);
        
        // Integral
        if (Math.abs(err) > INT_THRESHOLD) {
            integralSum = 0.0f;
            res[1] = 0.0f;
        } else {
            integralSum += err / (FREQUENCY / SLEEP_FREQ); // what matters is the accumulation
            res[1] = physicsOfPID(integralSum, err / (FREQUENCY / SLEEP_FREQ), PID.I);
        }
        
        // Differential
        res[2] = physicsOfPID(0, err - lastError, PID.D);
        lastError = err;
        
        return res;
    }
    
    private void updateMotors(float[] speeds) {
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
        
        // real limit is 6k, but it's reduced to limit hyperactive engines at the startup
        int iAcceleration = toDegrees((acceleration <= 4500) ? acceleration : 4500);
        
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
    
    enum PID { P, I, D};
    
    private float physicsOfPID(float angle, float d1theta_dt1, PID pid) {
        // second derivative of our angle
        double d2theta_dt2 = (3 * G_GRAVITY * Math.sin(angle))
                / (2 * ROBOT_LENGTH_CENTER_OF_MASS);
        
        // time frame over which we predict our movement
        double dt = ((double)FREQUENCY)/1000;
        
        // second integral over time
        // dt is a fixed time frame and we assume angular acceleration won't change
        double newAngle = angle +
                (d1theta_dt1 * dt) +
                (d2theta_dt2 * dt * dt) / 2;
        
        // projection on the level of rotation center
        double compensationDistance = ROBOT_LENGTH * Math.sin(newAngle);
        
        // compDist gives us unitary angle, so we adapt it to match actual wheels
        double compensationAngle = (compensationDistance / (ROBOT_WHEEL_RADIUS * Math.PI * 2));
        
        // calculate speed at which we have to move to cover this distance in dt time
        double requiredSpeed = compensationAngle / dt;
        
        // place for dirty hacks to be inserted
        switch (pid) {
        case P:
            if (Math.abs(newAngle) <= 0.2) {
                requiredSpeed *= 0.85;
            }
            break;
            
        case I:
            break;
            
        case D:
            if (Math.abs(newAngle) >= 0.25) {
                if (Math.abs(newAngle) >= 0.15) {
                    requiredSpeed *= 0.80;
                } else {
                    requiredSpeed *= 0.50;
                }
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
    
    private void lifespanFinishedTrigger() {
        gyro.close();
        motors[MOTOR_LEFT].close();
        motors[MOTOR_RIGHT].close();
    }
    
    private void checkLifespan() {
        if (fromStart >= 150000) {
            this.endOfLifeSpan = true;
        }
    }
    
    // static
    public static void main(String[] args) {
        EV3_Terminator ev3 = new EV3_Terminator();
        try {
            ev3.balancingLoop();
            System.out.println("EXIT_SUCCESS");
        }
        catch (Exception ex) {
            displayError(ex);
        }
    }
    
    private static void displayError(Exception ex) {
        System.out.println(ex.getMessage());
        ex.printStackTrace();
    }
    
}
