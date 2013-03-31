class pair
{
	double xVar;
	double yVar;

	//Constructors
	pair(double newX,double newY) { xVar = newX; yVar = newY; }
	
	//Setter and getter methods
	void setVarX(double newX) { xVar = newX; }
	void setVarY(double newY) { yVar = newY; }
	void setPair(double newX,double newY) {xVar = newX; yVar = newY; }
	
	double getVarX() { return xVar; }
	double getVarY() { return yVar; }
}

