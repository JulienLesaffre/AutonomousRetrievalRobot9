/**
 * 
 */
package ca.mcgill.ecse211.ARR;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.localizers.LightLocalizer;
import ca.mcgill.ecse211.localizers.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import java.util.Map;
import lejos.hardware.Button;

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

	//wifi connection parameters
	private static final String SERVER_IP = "192.168.2.1";
	private static final int TEAM_NUMBER = 9;
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;


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
	
	@SuppressWarnings("rawtypes")
	private static void retrieveDataFromServer() {
		
		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might occur
		try {
			
			//waits till start button pushed
			//can kill while waiting by pressing escape button
			Map data = conn.getData();

			//assign all the corresponding variables by retrieving the data
			Navigation.RedTeam = ((Long) data.get("RedTeam")).intValue();
			Navigation.GreenTeam = ((Long) data.get("GreenTeam")).intValue();
			Navigation.RedCorner = ((Long) data.get("RedCorner")).intValue();
			Navigation.GreenCorner = ((Long) data.get("GreenCorner")).intValue();
			Navigation.Red_LL_x = ((Long) data.get("Red_LL_x")).intValue();
			Navigation.Red_LL_y = ((Long) data.get("Red_LL_y")).intValue();
			Navigation.Red_UR_x = ((Long) data.get("Red_UR_x")).intValue();
			Navigation.Red_UR_y = ((Long) data.get("Red_UR_y")).intValue();
			Navigation.Green_LL_x = ((Long) data.get("Green_LL_x")).intValue();
			Navigation.Green_LL_y = ((Long) data.get("Green_LL_y")).intValue();
			Navigation.Green_UR_x = ((Long) data.get("Green_UR_x")).intValue();
			Navigation.Green_UR_y = ((Long) data.get("Green_UR_y")).intValue();
			Navigation.Island_LL_x = ((Long) data.get("Island_LL_x")).intValue();
			Navigation.Island_LL_y = ((Long) data.get("Island_LL_y")).intValue();
			Navigation.Island_UR_x = ((Long) data.get("Island_UR_x")).intValue();
			Navigation.Island_UR_y = ((Long) data.get("Island_UR_y")).intValue();
			Navigation.TNR_LL_x = ((Long) data.get("TNR_LL_x")).intValue();
			Navigation.TNR_LL_y = ((Long) data.get("TNR_LL_y")).intValue();
			Navigation.TNR_UR_x = ((Long) data.get("TNR_UR_x")).intValue();
			Navigation.TNR_UR_y = ((Long) data.get("TNR_UR_y")).intValue();
			Navigation.TNG_LL_x = ((Long) data.get("TNG_LL_x")).intValue();
			Navigation.TNG_LL_y = ((Long) data.get("TNG_LL_y")).intValue();
			Navigation.TNG_UR_x = ((Long) data.get("TNG_UR_x")).intValue();
			Navigation.TNG_UR_y = ((Long) data.get("TNG_UR_y")).intValue();
			Navigation.TR_x = ((Long) data.get("TR_x")).intValue();
			Navigation.TR_y = ((Long) data.get("TR_y")).intValue();
			Navigation.TG_x = ((Long) data.get("TG_x")).intValue();
			Navigation.TG_y = ((Long) data.get("TG_y")).intValue();

		} catch (Exception e) {
			//throws exception when: wrong IP, server not running, not connected to WIFI
			//also throws exception if: recieves bad data, message from server, e.g. make sure TEAM numb correct
			System.err.println("Error: " + e.getMessage());
		}
		// Wait until user decides to continue
		Button.waitForAnyPress();
	}
	
	
	

	public static void main(String[] args) throws OdometerExceptions {
		
		Display.displayStartScreen(); 
		
		initialize(); 									//initialize class variables needed
		
		retrieveDataFromServer();						//connect to the server and wait to recieve variables
		
		usLocalizer.fallingEdge();						//us localize
		
		lightLocalizer.localize(Navigation.RedCorner); 	//light localize
		
		Navigation.travelStartToTunnel();
		

	}

}
