/**
 * 
 */
package ca.mcgill.ecse211.ARR;

import ca.mcgill.ecse211.localizers.LightLocalizer;
import ca.mcgill.ecse211.localizers.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensors.LightController;
import ca.mcgill.ecse211.sensors.LightPoller;
import ca.mcgill.ecse211.sensors.USController;
import ca.mcgill.ecse211.sensors.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * @author JulienLesaffre
 * @author FouadBitar
 *
 */
public class AutonomousRetrievalRobot {
	

	
	static Odometer odometer = null;
	static Navigation nav = null;
	static USController usController;
	static UltrasonicPoller usPoller;
	static UltrasonicLocalizer usLocalizer;
	static LightController lightController;
	static LightPoller lightPoller;
	static LightLocalizer lightLocalizer;
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();



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

		nav = new Navigation(); 					//navigation must be after Odometer created
		usController = new USController();
		lightController = new LightController();
		usLocalizer = new UltrasonicLocalizer();
		lightLocalizer = new LightLocalizer();
		
		
		//us poller 
		SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
		SampleProvider usDistance = usSensor.getMode("Distance");                                           
		usPoller = new UltrasonicPoller(usDistance, usController);			
		
		//light poller
		SensorModes myColorLeft = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
		SampleProvider myColorSampleLeft = myColorLeft.getMode("Red");
		SensorModes myColorRight = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
		SampleProvider myColorSampleRight = myColorRight.getMode("Red");
		lightPoller = new LightPoller(myColorSampleLeft, myColorSampleRight, lightController);
		
	}
	

	
	/*
	 * Writing main in specific methods so that
	 * merging will not cause any issues
	 */
	public static void mainFouad() throws OdometerExceptions {
		
		Display.displayStartScreen(); 
		
		initialize(); 
		
		usPoller.start();
		usLocalizer.fallingEdge();
		//stop the us poller
		//start light poller and localize
		lightPoller.start();
		lightLocalizer.localize(Navigation.RedCorner); 

		
		
		//start loop of execution
		// 1) 
		
		
	}

	public static void main(String[] args) throws OdometerExceptions {
		
//		int buttonChoice;
//
//		do {
//			// clear the display
//			lcd.clear();
//
//			// ask the user whether the motor should drive in a square or float
//			lcd.drawString("READY ?", 5, 3);
//
//
//			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
//		} while (buttonChoice != Button.ID_ENTER);
//		
//		if (buttonChoice == Button.ID_ENTER) {
			
//			initialize(); 
			RingSet.testRingSet();
			System.exit(0);


			
//		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);

		

		

	}

}