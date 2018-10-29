package ca.mcgill.ecse211.localizers;

import ca.mcgill.ecse211.ARR.Navigation;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.sensors.DataController;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class LightLocalizer extends Thread {
	
  	private static final int MAX_DISTANCE = 30;
  	private static final int RIGHT_ANGLE = 90; 
  	private static final int FORWARD_SPEED = 60;
  	private static final int ROTATE_SPEED = 50;
  	private static final int LIGHTSENSOR_TO_ROBOTCENTER = 4;
  	private static final double BLACK_LINE = 0.18;
  	
	private Odometer odo;
	private Navigation navigator;
	private DataController dataCont;
  	private EV3LargeRegulatedMotor leftMotor;
  	private EV3LargeRegulatedMotor rightMotor;
	
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			Navigation navigator) throws OdometerExceptions{
		this.odo = Odometer.getOdometer();
		this.navigator = navigator;
		this.dataCont = DataController.getDataController();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	
	public void run() {
		double saveY = 0.0;
		double lightIntensity;
		//move forward till detect line 
		navigator.moveStraight(MAX_DISTANCE, FORWARD_SPEED, true, true);
		while(true) {
			lightIntensity = dataCont.getL();
			if (lightIntensity < BLACK_LINE) {
				Sound.beep();
				saveY = odo.getXYT()[1];
				break;
			}
		}
		
		navigator.moveStraight(saveY, FORWARD_SPEED, false, false); 	//move backward by y traveled
		navigator.turnRobot(RIGHT_ANGLE, ROTATE_SPEED, true, false);	//turn right 90 degrees and straight
		navigator.moveStraight(MAX_DISTANCE, FORWARD_SPEED, true, true); 	
		
		while(true) {
			lightIntensity = dataCont.getL();
			if (lightIntensity < BLACK_LINE) {
				Sound.beep(); 
				break;
			}
		}
		// move distance of sensor-robotCenter
		navigator.moveStraight(LIGHTSENSOR_TO_ROBOTCENTER, FORWARD_SPEED, true, false); 
		// turn left 90 degrees
		navigator.turnRobot(RIGHT_ANGLE, ROTATE_SPEED, false, false); 	
		// move forward by saved y value + sensor-robotCenter
		navigator.moveStraight((LIGHTSENSOR_TO_ROBOTCENTER + saveY), FORWARD_SPEED, true, false); 
	}
}
