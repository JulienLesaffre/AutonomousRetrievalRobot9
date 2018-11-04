/**
 * 
 */
package ca.mcgill.ecse211.ARR;

import ca.mcgill.ecse211.localizers.LightLocalizer;
import ca.mcgill.ecse211.localizers.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/*
//////////////
////TODO//////
 * check if turnto method works properly, it might do 360 turns because the odometer is already off and it thinks its heading another direction
/////////////
*/


/**
 * @author JulienLesaffre
 * @author FouadBitar
 *
 */
public class AutonomousRetrievalRobot {
	

	static Odometer odometer = null;
	static Navigation nav = null;
	static UltrasonicLocalizer usLocalizer;
	static LightLocalizer lightLocalizer;
	
	private static SampleProvider leftSampleProvider;
	private static SampleProvider rightSampleProvider;
	private static SampleProvider usSampleProvider;


	/**
	 * method to start and run odometer thread
	 * and initializes all the static class variables that the main program will use
	 * in the correct order
	 * 
	 * @throws OdometerExceptions
	 */
	@SuppressWarnings("resource")
	private static void initialize() throws OdometerExceptions{
		
		odometer = Odometer.getOdometer(Navigation.leftMotor, Navigation.rightMotor, Navigation.TRACK, Navigation.WHEEL_RAD);

		Thread odoThread = new Thread(odometer);
		odoThread.start();
		
		//light samplers
		SensorModes myColorLeft = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
		leftSampleProvider = myColorLeft.getMode("Red");
		SensorModes myColorRight = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
		rightSampleProvider = myColorRight.getMode("Red");
		//us sampler 
		SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
		usSampleProvider = usSensor.getMode("Distance");       

		nav = new Navigation(leftSampleProvider, rightSampleProvider, odometer); 
		
		//localizers
		usLocalizer = new UltrasonicLocalizer(usSampleProvider);
		lightLocalizer = new LightLocalizer(leftSampleProvider, rightSampleProvider);
		
	}
	

	
	/*
	 * Writing main in specific methods so that
	 * merging will not cause any issues
	 */
	public static void mainFouad() throws OdometerExceptions {
		
		Display.displayStartScreen(); 
		
		initialize(); 
		
		
		usLocalizer.fallingEdge();

		lightLocalizer.localize(Navigation.RedCorner); 

		
		
		
	}

	
	
	public static void main(String[] args) {
	}

}
