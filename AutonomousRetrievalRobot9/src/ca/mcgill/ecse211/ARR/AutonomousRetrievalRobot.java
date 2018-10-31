/**
 * 
 */
package ca.mcgill.ecse211.ARR;

import ca.mcgill.ecse211.localizers.LightLocalizer;
import ca.mcgill.ecse211.localizers.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryDisplay;
import ca.mcgill.ecse211.sensors.DataController;
import ca.mcgill.ecse211.sensors.LightColorPoller;
import ca.mcgill.ecse211.sensors.LightPoller;
import ca.mcgill.ecse211.sensors.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * @author JulienLesaffre
 *
 */
public class AutonomousRetrievalRobot {

	// Game parameters (will be provided with Wifi Class)
	public static final int RedTeam = 0; // Team starting out from red zone
	public static final int GreenTeam = 0; // Team starting out from green zone
	public static final int[] RedCorner = { 0, 0 }; // Starting corner for red team
	public static final int[] GreenCorner = { 0, 0 }; // Starting corner for green team
	public static final int[] Red_LL = { 0, 0 }; // lower left hand corner of Red Zone
	public static final int[] Red_UR = { 0, 0 }; // upper right hand corner of Red Zone
	public static final int[] Green_LL = { 0, 0 }; // lower left hand corner of Green Zone
	public static final int[] Green_UR = { 0, 0 }; // upper right hand corner of Green Zone
	public static final int[] BRR_LL = { 0, 0 }; // lower left hand corner of the red tunnel footprint
	public static final int[] BRR_UR = { 0, 0 }; // upper right hand corner of the red tunnel footprint
	public static final int[] BRG_LL = { 0, 0 }; // lower left hand corner of the green tunnel footprint
	public static final int[] BRG_UR = { 0, 0 }; // upper right hand corner of the green tunnel footprint
	public static final int[] TR_LL = { 0, 0 }; // lower left hand corner of the red player ring set
	public static final int[] TR_UR = { 0, 0 }; // upper right hand corner of the red player ring set
	public static final int[] TG_LL = { 0, 0 }; // lower left hand corner of the green player ring set
	public static final int[] TG_UR = { 0, 0 }; // upper right hand corner of the green player ring set

	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S1");

	public static void main(String[] args) throws OdometerExceptions {

		// Thread initialization
		Odometer odometer = Odometer.getOdometer(Navigation.leftMotor, Navigation.rightMotor, Navigation.TRACK,
				Navigation.WHEEL_RAD);
		OdometryDisplay odometryDisplay = new OdometryDisplay(lcd);
		Navigation navigation = new Navigation();
		UltrasonicLocalizer uLocalizer = new UltrasonicLocalizer(navigation);
		LightLocalizer lLocalizer = new LightLocalizer(navigation);
		DataController dataCont = DataController.getDataController();

		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
		SampleProvider usDistance = usSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];

		@SuppressWarnings("resource")
		SensorModes lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
		SampleProvider lightSample = lightSensor.getMode("Red");
		float[] lightData = new float[lightSensor.sampleSize()];

		@SuppressWarnings("resource")
		SensorModes ringSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
		SampleProvider ringSample = ringSensor.getMode("Red");
		float[] ringData = new float[lightSensor.sampleSize()];

		// Start odometer and sensor threads
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread usPoller = new UltrasonicPoller(usDistance, usData, dataCont);
		usPoller.start();
		Thread lightGPoller = new LightPoller(lightSample, lightData, dataCont);
		lightGPoller.start();
		Thread lightRPoller = new LightColorPoller(ringSample, ringData, dataCont);
		lightRPoller.start();

		int buttonChoice;
		do {
			lcd.clear(); // clear the display
			lcd.drawString("      ", 0, 0);
			lcd.drawString("Start ? ", 0, 1);
			lcd.drawString("      ", 0, 2);

			buttonChoice = Button.waitForAnyPress();

		} while (buttonChoice != Button.ID_ENTER);

		if (buttonChoice == Button.ID_ENTER) {
			// Start ultrasonic localizer thread and wait for it to finish
			uLocalizer.start();
			try {
				uLocalizer.join();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			// Start light localizer and wait for it to finish
			lLocalizer.start();
			try {
				lLocalizer.join();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			// start navigation

		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);

	}

}
