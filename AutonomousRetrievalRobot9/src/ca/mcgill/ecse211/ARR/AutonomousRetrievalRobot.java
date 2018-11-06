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
////////////////////////////////////
/////////////// TODO ///////////////
 * check if turnto method works properly, it might do 360 turns because the odometer is already off and it thinks its heading another direction
 * add median filter for the light sensors
 * able to receive values from wifi **
 * running slowly, increase speed, if cant its because too much power on cpu
 * turn off sensors when not needed i.e. ultrasonic (i.e. do we nullify the SampleProvider or what?
 * watch out traveltowithcorrection also does a turn to
 * can test out going diagonally to the side of the tunnel and then "localizing"
 * 
 * robot is placed in random orientation in corner, need to account for it starting at low edge
 * have to wait once placed in corner for wifi to pass parameters
 * beep 3 times after localizing
 * go to ring set and once you get there issue 3 beeps
 * when you detect the rings you issue beeps based on their number
 * when you are done unloading you issue 5 beeps
 * 
////////////////////////////////////
*/


/**
 * @author JulienLesaffre
 * @author FouadBitar
 * 
 * This is the main execution class for the robot.
 * 
 * There are no polling classes that create threads so as to minimize the CPU load. The SampleProviders
 * are initialized in the main and are sent to the classes that want to use it. This is so that we use same
 * initialization.
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
	

	public static void main(String[] args) throws OdometerExceptions {
		
		Display.displayStartScreen(); 
		
		initialize(); 	//initialize class variables needed
		
		usLocalizer.fallingEdge();						//us localize
		
		lightLocalizer.localize(Navigation.RedCorner); 	//light localize
		
		Navigation.travelStartToTunnel();
		
		Navigation.setSpeedAcceleration(200, 1500);
		Navigation.moveStraight(Navigation.SQUARE_SIZE/2, true, false);
		Navigation.turnTo(0);
		Navigation.moveStraight(Navigation.SQUARE_SIZE*3.5, true, false);
		
		
	}

}
