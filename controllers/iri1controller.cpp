/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>
#include <time.h>
#include <string>   
/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"

/**************** DEFINES ***********************/
/***********************************************/


#define DEBUG

#define N_BEHAVIOUR			4

#define BAT_THRES			0.5
#define FEAR_THRES		    0.7
#define DEAD_THRES			0.8
#define TRASH_THRES			0.65
#define AVOID_THRES			0.5

#define MORE_COFFEE_INDEX			0
#define GO_TRASH_INDEX				1
#define NAVIGATE_INDEX				2
#define AVOID_INDEX					3

//vTable array positions

#define ANGLE 						0
#define MOD 						1
#define ACTIVE 						2

//answers
#define YES							1
#define NO 							2

#define SPEED 						500.0

//ERRORS
#define ERROR_DIRECTION 0.05 
#define ERROR_POSITION  0.02


//ANSI
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

/************** LOCAL VARIABLES *****************/
/************************************************/

//Behaviour table
//		| angle	|  max	| active	
// bh1	|  x.x	| y.y 	|   1 .1 / 0.0
// bh2	| z.z	| w.w 	|	1 .1 / 0.0

double vTable[N_BEHAVIOUR][3]; 	
double lastVaue;
string state;
bool fStep = true;



using namespace std;

CIri1Controller::CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	/* Set Write to File */
	m_nWriteToFile = n_write_to_file;

	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set Blue light Sensor */
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
	/* Set Red light Sensor */
	m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);

	//inicializamos las variables

	nAdverts = 0;
	m_fRightSpeed = 0;
	m_fLeftSpeed = 0; 

	lastVaue = 0.0;
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	ExecuteBehaviors();
	Coordinator();
	m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);
	
	#ifdef DEBUG
		cout << state << "\n";
	#endif
}
/******************************************************************************/
/******************************************************************************/


void CIri1Controller::CleanTable(){
	system("clear");
	state =  "\t \t \tConsole state \n";
	cout << "\033[0;0H";


	for(int i= 0; i < N_BEHAVIOUR; i++){
		for (int c = 0; c < 3; c++)
		{
			vTable[i][c] = 0;
			if(c = 2)
				vTable[i][c] = 1;
		}
	}
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Navigate(){

	if(!vTable[NAVIGATE_INDEX][ACTIVE])
		return;

	vTable[NAVIGATE_INDEX][ANGLE] = 0.0;
	vTable[NAVIGATE_INDEX][MOD] = 0.3;
	vTable[NAVIGATE_INDEX][ACTIVE] = 1.0;

}
/******************************************************************************/
/******************************************************************************/

void CIri1Controller::avoid(){

	double res[2];
	IsItScary(SENSOR_PROXIMITY, YES, res);

	if(res[1] > AVOID_THRES){
		vTable[AVOID_INDEX][ANGLE] = res[0];
		vTable[AVOID_INDEX][MOD] = 1.0;
	}

}
/******************************************************************************/
/******************************************************************************/

/* Funcion auxiliar: Calcula el vector de repulsion en funcion de los datos de los sensores.
*	Param ->	SensorType:  Sensor de luz o proximidad.
*				table: Array donde se almacenan los resultados
*/
void CIri1Controller::IsItScary(int sensorType, int answer, double* table){
	double* rSensor;
	int nInputs;
	const double* rDirections;
	string sname ;

	if(sensorType == SENSOR_REAL_LIGHT){
		rSensor = m_seLight->GetSensorReading(m_pcEpuck);
		nInputs = m_seLight->GetNumberOfInputs();
		rDirections = m_seLight->GetSensorDirections();

		sname ="YELOW SENSOR: \n";

	}
	if(sensorType == SENSOR_REAL_BLUE_LIGHT){
		rSensor = m_seBlueLight->GetSensorReading(m_pcEpuck);
		nInputs = m_seBlueLight->GetNumberOfInputs();
		rDirections = m_seBlueLight->GetSensorDirections();
		sname ="BLUE SENSOR: \n";
	}
	if(sensorType == SENSOR_REAL_RED_LIGHT){
		rSensor = m_seRedLight->GetSensorReading(m_pcEpuck);
		nInputs = m_seRedLight->GetNumberOfInputs();
		rDirections = m_seRedLight->GetSensorDirections();
		sname ="RED SENSOR: \n";
	}
	
	if(sensorType == SENSOR_PROXIMITY){
			rSensor = m_seProx->GetSensorReading(m_pcEpuck);
			nInputs = m_seProx->GetNumberOfInputs();
			rDirections = m_seRedLight->GetSensorDirections();
		sname ="PROX SENSOR: \n";
	}

	#ifdef DEBUG
		state = state + sname;
	#endif
//Casi literal!.

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	double fMaxProx = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < nInputs ;  i ++ )
	{	
		#ifdef DEBUG
			state = state +  "sensor" + to_string(i) + ": "+to_string(rSensor[i])+" \t" ;
		#endif
		vRepelent.x += rSensor[i] * cos ( rDirections[i] );
		vRepelent.y += rSensor[i] * sin ( rDirections[i] );

		if ( rSensor[i] > fMaxProx )
			fMaxProx = rSensor[i];
	}
	#ifdef DEBUG
		state = state + "\n \n";
	#endif
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	if(answer == YES)
		/* Create repelent angle (Run, Forrest, Run!)*/
		fRepelent -= M_PI;

	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

  	table[0] = fRepelent;
  	table[1] = fMaxProx;

}
/******************************************************************************/
/******************************************************************************/
/*  Si la bateria esta descargada, busca la maquina de cafe mas cercana para recargarse.
*/
void CIri1Controller::MoreCofee(void){
	double* battery = m_seBattery->GetSensorReading(m_pcEpuck);

	if(battery[0] <= BAT_THRES){
		vTable[NAVIGATE_INDEX][ACTIVE] = 0.0;
		vTable[GO_TRASH_INDEX][ACTIVE] = 0.0;

		IsItScary(SENSOR_REAL_LIGHT, NO, vTable[MORE_COFFEE_INDEX]);
	}
	#ifdef DEBUG
		state += "Battery level: " + to_string(battery[0]) + "\n";
	#endif

}

/******************************************************************************/
/******************************************************************************/
void CIri1Controller::trash(void){
	//compruebo que la funcion no este inhibida
	if(!vTable[GO_TRASH_INDEX][ACTIVE])
		return;

	#ifdef DEBUG
		state +="Adverts: "+ to_string(nAdverts)+"\n";
	#endif
	//calculo el vector repulsivo
	if(nAdverts){
		vTable[NAVIGATE_INDEX][ACTIVE] = 0;
		IsItScary(SENSOR_REAL_BLUE_LIGHT, NO, vTable[GO_TRASH_INDEX]);
		if(vTable[GO_TRASH_INDEX][MOD] > TRASH_THRES){
			nAdverts = 0;
		}
	}
	else{
		double result[2];

		IsItScary(SENSOR_REAL_RED_LIGHT, YES, result);
		//Si nos hemos topado con un vendedor ( DEAD_THRES ), cogemos su publicidad y disimuladamente la reciclamos.

		if(result[1] >= DEAD_THRES)
			nAdverts++;
		else{
			//Si podemos evitarlo, lo hacemos y corremos  en direccion contraria
			if(result[1] >= FEAR_THRES){
				vTable[GO_TRASH_INDEX][ANGLE] = result[0];
				vTable[GO_TRASH_INDEX][MOD]	=	result[1];
			}
		}
	}

}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ExecuteBehaviors (void){

	CleanTable();
	avoid();
	MoreCofee();
	trash();
	Navigate();

};

void CIri1Controller::Coordinator(){
	int nBehavior;
	double fAngle = 0.0;

	dVector2  vAngle;
	vAngle.x = 0.0;
	vAngle.y = 0.0;

  /* For every Behavior */
	for ( nBehavior = 0 ; nBehavior < N_BEHAVIOUR ; nBehavior++ )
	{
		#ifdef DEBUG 
			state = state + "Behavior "+ to_string(nBehavior)+"->\t"  +"Angle: " + to_string(vTable[nBehavior][ANGLE])+"\tMod: " + to_string(vTable[nBehavior][MOD]) +"\tactive: "+ to_string(vTable[nBehavior][ACTIVE])+" \n";
		#endif 

		/* If behavior is active */

		if (vTable[nBehavior][ACTIVE])
		{
     		vAngle.x += vTable[nBehavior][MOD] * cos(vTable[nBehavior][ANGLE]);
    		vAngle.y += vTable[nBehavior][MOD] * sin(vTable[nBehavior][ANGLE]);
		}
	}

  /* Calc angle of movement */
  fAngle = atan2(vAngle.y, vAngle.x);
  
  if (fAngle > 0)
  {
    m_fLeftSpeed = SPEED*(1 - fmin(fAngle, ERROR_DIRECTION)/ERROR_DIRECTION);
    m_fRightSpeed = SPEED;
  }
  else
  {
    m_fLeftSpeed = SPEED;
    m_fRightSpeed = SPEED*(1 - fmin(-fAngle, ERROR_DIRECTION)/ERROR_DIRECTION);
  }
  #ifdef DEBUG
  	state = state +  "Left Speed: "+ to_string(m_fLeftSpeed)+ "Right Speed: "+ to_string(m_fRightSpeed)+ "\n";
  #endif
}
/******************************************************************************/
/******************************************************************************/

