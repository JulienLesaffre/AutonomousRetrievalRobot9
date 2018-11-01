package ca.mcgill.ecse211.ARR;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

/**
 * Should not be thread, we should use this class to debug only 
 * and call on it from other methods if we want to display something
 */
public class Display {

	public static final TextLCD lcd = LocalEV3.get().getTextLCD();

	/**
	 * This is the class constructor
	 * 
	 * @param lcd
	 * @throws OdometerExceptions 
	 */
	public Display() {
	}

	/**
	 * this method asks user to start
	 * must press right button to start then clears display and executes prog
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
	 * this method is used to debug us localization
	 */
	public static void displayUSLocalization(int d, double a, double b, double result) {
		lcd.clear();
		lcd.drawString("Distance: " + d, 0, 1);
		lcd.drawString("Alpha: " + a, 0, 2);
		lcd.drawString("Beta: " + b, 0, 3);
		lcd.drawString("Final: " + result, 0, 4);
	}
	
	
	public static void displayLightLocalization(float left, float right, double x, double y, double theta) {
		lcd.clear();
		lcd.drawString("colorLeft: " + left, 0, 1);
		lcd.drawString("colorRight: " + right, 0, 2);
		lcd.drawString("x: " + x, 0, 3);
		lcd.drawString("y: " + y, 0, 4);
		lcd.drawString("theta: " + theta, 0, 5);
	}







}
