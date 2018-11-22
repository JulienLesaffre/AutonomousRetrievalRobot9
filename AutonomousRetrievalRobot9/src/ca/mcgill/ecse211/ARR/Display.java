package ca.mcgill.ecse211.ARR;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to create specific static methods for displaying 
 * things on the lcd screen. Lcd is static variable initialized during 
 * compile so as not to mix things up.
 */
public class Display {

	public static final TextLCD lcd = LocalEV3.get().getTextLCD();


	/**
	 * Asks the user to start, must press right button to start then clears display and executes prog
	 */
	public static void displayStartScreen() {
		lcd.clear();
		int buttonChoice;
		do {
			lcd.drawString("Start ? ", 0, 0);
			buttonChoice = Button.waitForAnyPress(); 
		} while (buttonChoice != Button.ID_RIGHT);
		lcd.clear();
	}
	
	/**
	 * this method is used to display the parameters for the ultrasonic localization.
	 * @param d : distance
	 * @param a : alpha - the first falling edge
	 * @param b : beta - the second falling edge
	 * @param result : the final angle
	 */
	public static void displayUSLocalization(int d, double a, double b, double result) {
		lcd.clear();
		lcd.drawString("Distance: " + d, 0, 1);
		lcd.drawString("Alpha: " + a, 0, 2);
		lcd.drawString("Beta: " + b, 0, 3);
		lcd.drawString("Final: " + result, 0, 4);
	}
	
	
	/**
	 * this method is used to display the parameters for the light localization.
	 * @param left : left light sensor data
	 * @param right : right light sensor data
	 * @param x : x coordinate
	 * @param y : y coordinate
	 * @param theta : angle
	 */
	public static void displayLightLocalization(float left, float right, double x, double y, double theta) {
		lcd.clear();
		lcd.drawString("colorLeft: " + left, 0, 1);
		lcd.drawString("colorRight: " + right, 0, 2);
		lcd.drawString("x: " + x, 0, 3);
		lcd.drawString("y: " + y, 0, 4);
		lcd.drawString("theta: " + theta, 0, 5);
	}
	
	/**
	 * Displays odometer values on lcd screen.
	 * @param x : 
	 * @param y
	 * @param theta
	 */
	public static void displayNavigation(double x, double y, double theta) {
		lcd.clear();
		lcd.drawString("x: " + x, 0, 0);
		lcd.drawString("y: " + y, 0, 1);
		lcd.drawString("theta: " + theta, 0, 2);
	}
	
	
	public static void displayRingColor(int color) {
		lcd.clear();
		lcd.drawString("color: " + color, 0, 1);
	}


}
