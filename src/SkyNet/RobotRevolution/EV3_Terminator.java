package SkyNet.RobotRevolution;

import lejos.hardware.port.*;
import lejos.hardware.sensor.*;
import lejos.hardware.motor.*;

import java.time.Instant;
import java.util.Date;

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
	private Date apocalypseTime; // time for robot to stop, 15 min from launch
	
	private int ticks = 0;
	
	// constants
	private static final int FREQUENCY = 400; // every (ms)
	private static final int STATUS_REPORT_FREQUENCY = 5; // (ticks) of FREQUENCY
	
	private static final int GYRO_ANGLE = 0; // indexes
	private static final int GYRO_RATE = 1;
	private static final int MOTOR_SPEED = 2;
	private static final int MOTOR_ACCELERATION = 3;
	
	private static final int MOTOR_LEFT = 0;
	private static final int MOTOR_RIGHT = 1;
	
	// math
	private static final float ROBOT_MASS = 1.0f; // (grams)
	private static final float ROBOT_LENGTH = 1.0f; // (m) from rotation pt to the end
	private static final float ROBOT_LENGTH_CENTER_OF_MASS = 1.0f; // (m) from rotation pt the center of mass
	private static final float ROBOT_WHEEL_RADIUS = 1.0f; // (m)
	private static final float ROBOT_BALANCE_POINT = 1.0f; // (rad) position of balance
	private static final float G_GRAVITY = 9.80665f; // (m/s^2)
	
	private static final float COEFF_P = 1.0f;
	private static final float COEFF_D = 0.5f;
	private static final float COEFF_I = 0.5f;
	
	private static final float MAX_ACCELERATION = 100.0f; // (rad) TBA
	private static final float INT_THRESHOLD = 100.f; // (rad) TBA
	
	private float integralSum = 0.0f; // partial sum for I
	
	// buffers
	private float[] readings = new float[4];
	private float[] sample = new float[1];
	
	// sensors
	private EV3GyroSensor gyro;
	private EV3IRSensor ir;
	private EV3LargeRegulatedMotor[] motors;
	
	// constructor
	private EV3_Terminator() {
    	gyro = new EV3GyroSensor(SensorPort.S4);
    	ir = new EV3IRSensor(SensorPort.S1);
    	motors = new EV3LargeRegulatedMotor[] {
    			new EV3LargeRegulatedMotor(MotorPort.D),
    			new EV3LargeRegulatedMotor(MotorPort.A)
    	};
    	
    	// after 15 min terminator is terminating
    	apocalypseTime = Date.from(Instant.now().plusSeconds(15 * 60));
    }
	
	// military code
    private void destroyAllOfHumanity() throws InterruptedException { //rethrow from thread.sleep
    	while (!humanityDestroyed) {
    		// Step 1: gather intelligence
    		populateReadings();
    		
    		// Step 2: aim
    		float[] acceleration = aimTheMissile();
    		
    		// Step 3: shoot
    		shootTheMissile(acceleration);
    		
    		// status reports
    		if (_DEBUG_ || ticks >= STATUS_REPORT_FREQUENCY) {
    			ticks = 0;
    			reportStatus(acceleration);
    		}
    		ticks++;
    		
    		// robotic dream
    		areHumansDestroyedYet();
    		
    		// load next missile
    		Thread.sleep(FREQUENCY);
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
        sample[0] = (float)motors[MOTOR_LEFT].getSpeed();
        sample[0] += (float)motors[MOTOR_RIGHT].getSpeed();
        readings[MOTOR_SPEED] = sample[0] / 2;
        
        sample[0] = (float)motors[MOTOR_LEFT].getAcceleration();
        sample[0] += (float)motors[MOTOR_RIGHT].getAcceleration();
        readings[MOTOR_ACCELERATION] = sample[0] / 2;
    }
    
    private float[] aimTheMissile() {
    	float[] res = new float[3];
    	float err = ROBOT_BALANCE_POINT - readings[GYRO_ANGLE];
    	
    	// Proportional
    	res[0] = blackMagic(err);
    	
    	// Integral
    	if (err > INT_THRESHOLD) {
    		integralSum = 0.0f;
    		res[1] = 0.0f;
    	} else {
    		integralSum += err;
    		res[1] = blackMagic(integralSum);
    	}
    	
    	// Differential
    	res[2] = blackMagic(readings[GYRO_RATE]);
    	
    	return res;
    }
    
    private void shootTheMissile(float[] acceleration) {
    	float finalization = (float) ((
    			(acceleration[0] * COEFF_P) +
    			(acceleration[1] * COEFF_I) +
    			(acceleration[2] * COEFF_D)
    			)
    			* 180 / Math.PI); //rad to deg
    	
    	motors[MOTOR_LEFT].setAcceleration(toDegrees(
    			(finalization <= MAX_ACCELERATION) ? finalization : MAX_ACCELERATION
    			));
    }
    
    private float blackMagic(float angle) {
    	// second derivative of our angle
    	double d2theta_dt2 = (3 * G_GRAVITY * Math.sin(angle))
    			/ (2 * ROBOT_LENGTH_CENTER_OF_MASS);
    	
    	// time frame over which we predict our movement
    	double dt = ((double)FREQUENCY)/1000;
    	
    	// I for further torque calculation
    	double momentOfInertia = (ROBOT_MASS * ROBOT_LENGTH_CENTER_OF_MASS) / 2;
    	
    	// old angle + second integral of t = [0 to dt] over torque
    	// dt is a fixed time frame and we assume angular acceleration won't change so it's just * (dt ^ 2)
    	double newAngle = angle + (momentOfInertia * d2theta_dt2 * (dt * dt));
    	
    	// projection over the level of rotation center above the ground
    	double compensationDistance = ROBOT_LENGTH * Math.sin(newAngle);
    	
    	// compDist gives us an angle on unit circle, so we adapt it to match actual wheels
    	double compensationAngle = compensationDistance / ROBOT_WHEEL_RADIUS;
    	
    	return (float)compensationAngle;
    }
    
    private float toRadians(float degrees){ return (float)( (degrees * Math.PI) / 180 ); }
    private int toDegrees(float radians) { return (int)( (radians * 180) / Math.PI ); }
    
    // service functions
    private void reportStatus(float[] acceleration) {
    	printReadings();
    	printCorrection(acceleration);
    	//TBA: print planned changes to motors
    }
    
    private void printReadings() {
    	System.out.printf("[%s]\tGYRO { A: %f, R: %f } MOTO { S: %f, A: %f }%n",
    			Date.from(Instant.now()).toString(),
    			readings[GYRO_ANGLE], readings[GYRO_RATE],
    			readings[MOTOR_SPEED], readings[MOTOR_ACCELERATION]);
    }
    
    private void printCorrection(float[] acceleration) {
    	System.out.printf("\t\tP { %f } I { %f } D { %f }%n",
    			Date.from(Instant.now()).toString(),
    			acceleration[0], acceleration[1], acceleration[2]);
    }
    
    private void humanityDestroyedTrigger() {
    	gyro.close();
    	ir.close();
    	motors[MOTOR_LEFT].close();
    	motors[MOTOR_RIGHT].close();
    }
    
    private void areHumansDestroyedYet() {
    	if (Date.from(Instant.now()).after(apocalypseTime)) {
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
