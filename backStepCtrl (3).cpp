// backstepping

class BackStepCtrl
{
	// all repeatedly used data should be stored as class members


public:
	float m_pitchDes;
	float m_rollDes;
	float m_thrustDes;
	float rpt;
    float integratorMin;
    float integratorMax;
    float lamda=5.0;
    float m_mass = 9;
    float m_g =9.8;
    float xd, //x_desired
    float xa, //x_actual
    float yd, //y_desired
    float ya, //y_actual
    float zd, //z_desired
    float za, //z_actual
    float m_mass;
    const std::string& name)
        : m_kr(kr)
       // , m_kd(kd)
        , m_ki(ki)
        , m_minforce(minforce)
        , m_maxforce(maxforce)
        , m_integratorMin(integratorMin)
        , m_integratorMax(integratorMax)
        , m_integral(0)
        , m_previousError(0)
        , m_previousTime(ros::Time::now())
    {
    }


   public:
	// constructor
	BackStepCtrl():
	
	m_pitchDes(0.0),
	m_rollDes(0.0);
	m_thrustDes(0.0);
	m_force;
	{
		std::vector<float> dummy(3, 5.0);
		lambda = dummy;

		std::vector<float>dummy<3,9.8>
		m_g = dummy;
		// initializing more complex epressions 
	}
public:
	void reset()
    {
        m_integral = 0;
        m_previousError = 0;
        m_previousTime = ros::Time::now();
    }

    void setIntegral(float integral)
    {
        m_integral = integral;
    }

    float ki() const

    {
        return m_ki;
    }

public:
	std::vector<float> GetDesired(){
		// calculates and returns desired roll, pith, thrust (RPT)
		this->CalcDesired();

		std::vector<float> output;
		output.push_back(m_pitchDes)  ; 
		output.push_back(m_rollDes)   ;
		output.push_back(m_thrusthDes);
		return output;
	}

	 double diffFunc(double x, int n)
    {
        double f =1.0;
        for(int i=0;i<n;i++)
        {
            f* =x;
            return f;
        }
    }

     double transposeMatrix(double Array[],int length) // for getting transpose of matrix
     {
        int i=0;
        double temp[length][i];
        for(int j=0;j<length;j++)
        {
          temp[j][i] = Array[i][j];
        
        }

        return temp;
     }

      double calcR (double Array[],length)   //calculating r matrix i.e r = e+lamda*e

     {
        double matrix[length] ;
        int i=0;
        for(int j=0;i<length;i++)
        {
          matrix[j][i]=Array[j][i]+ lamda*(Array[j][i]);
        }
        return matrix;
     }


private:
	void CalcDesired(){
		this->CalcForce();
		this->CalcRPT();
	}
private:
	 double update_force(vector<double>doubleDd , double error[],double rmatrix[], ) 

    {
        ros::Time time = ros::Time::now();
        double dt = time.toSec() - m_previous
        double forceCalc[3];
        // initialize as std::vector private members
        Time.toSec();
       
      for(int i=0;i<3;i++)
       if (dt > 0)
        {
            
            forceCalc(i) = m_mass*doubleDd.at(i) - m_mass*lamda*lamda*error(i) - m_mass*m_g + kr*rmatrix(i)+ ki*m_integral
        }
        
        m_previousTime = time;
        
        // self.pubOutput.publish(force)
        // self.pubError.publish(r1)
        // self.pubP.publish(e1)
        // self.pubI.publish(i)
        return std::max(std::min(forceCalc, m_maxforce), m_minforce);
    }

	void CalcRPT(){
		m_force;

		m_pitchDes;
		m_rollDes;
		m_thrusthDes;
    tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

     geometry_msgs::Wrench msg;
                msg.m_force.x = m_pidX.forceCalc( targetDrone.pose.position.x); ///dont know how to display these lines
                msg.m_force.y = m_pidY.forceCalc(,targetDrone.pose.position.y); ///dont know how to display these lines
                msg.m_force.z = m_pidZ.forceCalc( targetDrone.pose.position.z); ///dont know how to display these lines
                //msg.angular.z = m_pidYaw.update(0.0, yaw);
                m_pubNav.publish(msg);
                
                 float a = - m_force.x;
                 float b1= m_force.y*m_force.y+m_force.z*m_force.z;
                 float b= pow(b1,0.5);
                 float m_thrustDes = sqrt(a*a+ b*b);
                 float m_pitchDes= atan(a/b);
                 float m_rollDes = atan(m_force.y/m_force.x);
                 
                 BackStepCtrl CTRL();
                geometry_msgs::Twist msg;
                rpt = CTRL.CalcDesired()
                  msg.linear.x = m_rollDes; //rpt[0]
                  msg.linear.y = m_pitchDes; // rpt[1]
                  msg.linear.z = m_thrustDes; // rpt[2]
                   m_pubNav.publish(msg);           


	}
     int main()
{
    double vector<double> desiredAct; //storing desired nd actual value of sigma from user
    double vector<double> doubleDiffernt //vector for storing 2nd order differntiative  value
    double a,b;
    double e1,e2,e3; // variables for storing error x,y,z co-ordinates respectively
    double errorTranspose[3];
    double rMatrix[3];

    cout<<"please enter values for xDesired, xActual, yDesired, yActual, zDesired, zActual"<<endl;
    for(int i=0;i<6;i++)
    {
        cin>> a; //input from user
        desiredACt.push_back(a); // xDesired, xActual, yDesired, yActual, zDesired, zActual

    }
    for(int i=0;i<6;i++) //length =6 because we just have 6 elements i.e. xDesired, xActual, yDesired, yActual, zDesired, zActual
    {
       b= func(dsiredAct.at(i),2);
        doubleDiffernt.push_back(b); //Calculating double differentiation of xDesired, xActual, yDesired, yActual, zDesired, zActual values
    }
    e1= doubleDiffernt(0)-doubleDiffernt(1); //calculating x-coordinate of error 'e'
    e2= doubleDiffernt(2)-doubleDiffernt(3); //calculating y-coordinate of error 'e'
    e3= doubleDiffernt(4)-doubleDiffernt(5); //calculating z-coordinate of error 'e'
     double errorMatrix[3]={e1,e2,e3}; // array for storing error values
    double vector<double> desiredDifferent={doubleDiffernt(0),doubleDiffernt(2),doubleDiffernt(4)} ; //creating vector for sigma desired double differentiate
   errorTranspose = transposeMatrix(errorMatrix,3 ) // calculating error by doing transpose of original matrix
   rMatrix = calcR( errorTranspose, 3); //calculting r using r = error+lamda*error
    update_force(vector<double>desiredDifferent , errorTranspose,rMatrix); // updating force
}
	