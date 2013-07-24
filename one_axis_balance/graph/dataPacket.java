class dataPacket
{
	//Data members
	long time;
	float [] ypr = new float[3];
	float [] pid = new float[3];
	float [] motorValues = new float[4];
	
	//Constructors
	dataPacket()
	{
		time=0;
		ypr[0]=0;
		ypr[1]=0;
		ypr[2]=0;
		pid[0] = 0;
		pid[1] = 0;
		pid[2] = 0;
		motorValues[0] = 0;
		motorValues[1] = 0;
		motorValues[2] = 0;
		motorValues[3] = 0;
	}
	
	dataPacket(long timeInit, float yaw, float pitch, float roll, float p, float i, float d, float MV1, float MV2, float MV3, float MV4)
	{
		time = timeInit;
		ypr[0] = yaw;
		ypr[1] = pitch;
		ypr[2] = roll;
		pid[0] = p;
		pid[1] = i;
		pid[2] = d;
		motorValues[0] = MV1;
		motorValues[1] = MV2;
		motorValues[2] = MV3;
		motorValues[3] = MV4;
	}
	
	//Setter methods
	void setTime(long timeNew) { time = timeNew; }
	
	void setYawPitchRoll(float yaw, float pitch, float roll)
	{
		ypr[0] = yaw;
		ypr[1] = pitch;
		ypr[2] = roll;
	}
	
	void setYawPitchRoll(float [] yawPitchRoll)
	{
		if(yawPitchRoll.length < 3)
		{
			System.out.println("Error in setYawPitchRoll(float [] yawPitchRoll), array length less than three!");
			System.exit(0);
		}
		
		System.arraycopy(yawPitchRoll,0,ypr,0,3);
	}
	
	void setPID(float newP,float newI,float newD)
	{
		pid[0] = newP;
		pid[1] = newI;
		pid[2] = newD;
	}
	
	//Getter methods
	long getTime() { return time; }
	float getYaw() { return ypr[0]; }
	float getPitch() { return ypr[1]; }
	float getRoll() { return ypr[2]; }
	void getYawPitchRoll(float [] yawPitchRoll)
	{
		if(yawPitchRoll.length < 3)
		{
			System.out.println("Error in getYawPitchRoll(float [] yawPitchRoll), array length less than three!");
			System.exit(0);
		}
		
		System.arraycopy(ypr,0,yawPitchRoll,0,3);
	}
	float getYawPitchRoll(int valueToSelect)
	{
		if((valueToSelect >= 0) && (valueToSelect <= 2)) return ypr[valueToSelect];
		else
		{
			System.out.println("Error in getYawPitchRoll(int valueToSelect), value outside allowable range!");
			System.exit(0);
			return (float) 0.0;
		}
	}
	float getP() { return pid[0]; }
	float getI() { return pid[1]; }
	float getD() { return pid[2]; }
	void getMotorValues(float [] motorValues)
	{
		if(motorValues.length < 4)
		{
			System.out.println("Error in getMotorValues(float [] motorValues), array length less than 4");
			System.exit(0);
		}
	}
	float getMotorValues(int valueToSelect)
	{
		if((valueToSelect >= 0) && (valueToSelect <= 3)) return motorValues[valueToSelect];
		else
		{
			System.out.println("Error in getMotorValues(int valueToSelect), value outside allowable range!");
			System.exit(0);
			return (float) 0.0;
		}
	}
}
