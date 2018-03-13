#include "Copter.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

using namespace std;
//const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for IDS
 */
/*Copter::Copter(void)
    : DataFlash(fwver.fw_string, g.log_bitmask),
    flight_modes(&g.flight_mode1),
    control_mode(STABILIZE),
    scaleLongDown(1),
    simple_cos_yaw(1.0f),
    super_simple_cos_yaw(1.0),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),
    inertial_nav(ahrs),
    param_loader(var_info),
    flightmode(&mode_stabilize)
{
    // init sensor error logging flags
    sensor_health.baro = true;
    sensor_health.compass = true;
}

Copter copter;
*/



void Copter::IDS_setup()
{
    startWritingLogs = false; //ekta added
    totalDistanceTravelledTillNow = 0.0; // ekta added
    previousDistanceTravelledInThisTrack = 0.0; // ekta added
    attackStartAfterXSeconds = 10; //ekta added
    hasAttackCountdownBegun = false; // ekta added
    hasReadInvariantsFile = false; // ekta added
    errorTolerated = 0.2;
}


void Copter::write_ids_custom_logs(int32_t my_baro_alt, uint16_t my_battery,float acc,float dist,uint32_t speed, uint32_t course)
{
     // Generate invariants and write into file
     std::ofstream myfile;
     myfile.open ("physical_logs.txt",std::ios_base::app);

     
     if ((my_baro_alt <= 0 && startWritingLogs == false))
     {
      return;
     }
     else if (my_baro_alt > 0)
     {
     startWritingLogs = true;
     }
     if (dist > 0)
     {
     
     if (dist < previousDistanceTravelledInThisTrack)
     {
	previousDistanceTravelledInThisTrack = 0;
	totalDistanceTravelledTillNow = totalDistanceTravelledTillNow + dist - previousDistanceTravelledInThisTrack;
        previousDistanceTravelledInThisTrack = dist;
     }
     else
     {
	totalDistanceTravelledTillNow = totalDistanceTravelledTillNow + dist - previousDistanceTravelledInThisTrack;
        previousDistanceTravelledInThisTrack = dist;
     }
     }

#if ATTACK == ENABLED
// battery tampering attack, where after "attackStartAfterXSeconds" seconds, faulty battery value
// will be sent to GCS. This attack can be detected on drone and GCS as well.
     if (my_baro_alt > 0)
     {
	if (hasAttackCountdownBegun == false)
		{
			hasAttackCountdownBegun = true;
			flightStartTime = chrono::system_clock::now();
		}
     }
        chrono::system_clock::time_point currentTime = chrono::system_clock::now();
	chrono::duration<double> elapsed_seconds = currentTime - flightStartTime ;

	if (elapsed_seconds.count() > attackStartAfterXSeconds)
	{
		my_battery = faultyBatteryValue;
	}
	else
	{
		faultyBatteryValue = my_battery;
	}

#endif
     
     myfile <<"altitude"<<": "<<my_baro_alt<<"\n";
     myfile <<"battery"<<": "<<my_battery<<"\n";
     myfile <<"accel"<<": "<< acc<<"\n"; // in m/s square
     myfile <<"distance"<<": "<< (totalDistanceTravelledTillNow/100)<<"\n";
     myfile <<"speed"<<": "<< speed<<"\n";
     myfile <<"course"<<": "<< course<<"\n";
     myfile.close();
}

void Copter::ids_detect_intrusion()
{
     if (hasReadInvariantsFile == false)
     {
        // Read the physical invariants file "NonFaultyPhysicalInvariants.txt"
        std::ifstream file("NonFaultyPhysicalInvariants.txt");
        std::string str;
	std::string firstLine;
	std::getline(file, firstLine);
        char s[256];
        // seperate string by comma to get all physical variables
	std::stringstream ss (firstLine);
	while(ss.getline(s, 256, ',')) {
		phyVariables.push_back(std::string(s));

        }
	
        while (getline(file,str))
        {
		std::istringstream iss(str);
		float val;
		iss>>val;
		nonFaultyInvariants.push_back(val);
        }
        hasReadInvariantsFile = true;
	file.close();
     }
     generate_invariants_from_current_logs();
}

float sum(vector<float> a)
{
	float s = 0;
	for (int i = 0; i < a.size(); i++)
	{
		s += a[i];
	}
	return s;
}

float squaresum(vector<float> a)
{
	float s = 0;
	for (int i = 0; i < a.size(); i++)
	{
		s += pow(a[i], 2);
	}
	return s;
}

vector<float> operator-(vector<float> a, float b)
{
	vector<float> retvect;
	for (int i = 0; i < a.size(); i++)
	{
		retvect.push_back(a[i] - b);
	}
	return retvect;
}

vector<float> operator*(vector<float> a, vector<float> b)
{
	vector<float> retvect;
	for (int i = 0; i < a.size() ; i++)
	{
		retvect.push_back(a[i] * b[i]);
	}
	return retvect;
}

float stdev(vector<float> v)
{
	float N = v.size();
	return pow(squaresum(v) / N - pow(sum(v) / N, 2), 0.5);
}


float Copter::calculateCorrelationCoefficient(vector<float> vector1, vector<float> vector2)
{

	//Calculate pearson correlation coefficient for the two vectors.
	float temp = (vector1.size()*stdev(vector1)* stdev(vector2));
	if (temp == 0.0)
	{
		return 0.0;
	}
	return sum((vector1 - (sum(vector1)/vector1.size()))*(vector2 - (sum(vector2)/vector2.size()))) / temp;

}

void Copter::generate_invariants_from_current_logs()
{
      // Read the physical variables from file "physical_log.txt"
      std::ifstream file("physical_logs.txt");

      std::string line;

      char s[256];
      std::map<std::string,std::vector<float>> logData;

      std::string key, value;
      float newValue;

      while (getline(file,line))
       {
	// read the file and make dictionary structure out of it.
		std::stringstream ss (line);
		ss.getline(s, 256, ':');
		key = std::string(s);
		ss.getline(s, 256, ':');
		value = std::string(s);
		newValue = std::stof(value);

		if(logData.find(key) == logData.end() ) {
			logData.insert( std::make_pair(key, std::vector<float> ()));
		}
		logData[key].push_back(newValue);

       }
	// We have parsed the data and now we will generate invariants from it. 
     std::ofstream newfile;
     newfile.open ("physical_logs_result.txt",std::ios_base::app);

     
	float relation = 0.0;
	int index = 0;
	for (int i = 0 ; i < phyVariables.size() - 1 ; i++)
	{
		for (int j = i + 1; j < phyVariables.size() ; j++, index ++)
		{
				if((logData.find(phyVariables[i]) != logData.end()) && (logData.find(phyVariables[j]) != logData.end())) 
				{
					
			
					relation = calculateCorrelationCoefficient((logData.find(phyVariables[i]))->second, (logData.find(phyVariables[j]))->second);
					if ((nonFaultyInvariants[index] - errorTolerated <= relation) && (nonFaultyInvariants[index] + errorTolerated >= relation))
					{
						newfile << "No intrusion" << "\n";
					}
					else
					{
						newfile << "Intrusion" << "\n";
					}
				
				}
		}
	}
	newfile<<"-------------------------------------------\n";
	newfile.close();

}







