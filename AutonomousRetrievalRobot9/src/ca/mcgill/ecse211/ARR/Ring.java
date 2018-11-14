package ca.mcgill.ecse211.ARR;

public class Ring {
	
	boolean isUp;
	int ringColor;
	enum Side {
		Null, North, East, West, South;
	}
	Side ringSide;
	
	
	public Ring(boolean up, int color, Side side) {
		isUp = up;
		ringColor = color;
		ringSide = side;
	}
	
}
