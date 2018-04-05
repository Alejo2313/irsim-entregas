#ifndef IRI1CONTROLLER_H_
#define IRI1CONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"




/******************************************************************************/
/******************************************************************************/

class CIri1Controller : public CController
{
public:

    CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);
    ~CIri1Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:

	/* variables*/

	CEpuck* m_pcEpuck;
	CWheelsActuator* m_acWheels;
    CEpuckProximitySensor* m_seProx;
	CRealLightSensor* m_seLight;
	CRealBlueLightSensor* m_seBlueLight;
	CRealRedLightSensor* m_seRedLight;
	CContactSensor* m_seContact;
	CGroundSensor* m_seGround;
	CGroundMemorySensor* m_seGroundMemory;
	CBatterySensor* m_seBattery;  
	CBlueBatterySensor* m_seBlueBattery;  
	CRedBatterySensor* m_seRedBattery;  

	int nAdverts;
	int m_nWriteToFile;

	double m_fLeftSpeed;
	double m_fRightSpeed;

	/*FUNCTIONS*/

	void ExecuteBehaviors ( void );
	void Coordinator ( void );
	
	void CleanTable();
	void Navigate();
	void avoid();
	void IsItScary(int sensorType, int answer, double* table);
	void MoreCofee(void);
	void trash(void);

};

#endif
