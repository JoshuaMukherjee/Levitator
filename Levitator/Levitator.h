#pragma once
#include <AsierInho_V2.h>
#include <AsierInho.h>
#include <stdio.h>
#include <conio.h>
#include <Windows.h>
#include <stdio.h>

class Levitator {
	
	int numBoards;
	float* matBoardToWorld;
	int* boardIDs;
	int numTransducers;
	bool print;

	float* transducerPositions;
	float* transducerNormals;
	float* amplitudeAdjust;
	int* mappings;
	int* phaseDelays;
	int numDiscreteLevels;
	int num_geometries;

	unsigned char* phases_disc;
	unsigned char* amplitudes_disc;


public:
	AsierInho_V2::AsierInhoBoard_V2* driver;
	AsierInho::AsierInhoBoard* disc;
	Levitator( int* boardIDsIn, float* matBoardToWorldIn, int numBoardsIn = 2,bool printIn=true);
	//Levitator();
	int sendMessages(float* phases, float* amplitudes, float relative_amp=1, int num_geometriesIn =1, int sleep_ms=0);
	int TurnOff();
	int Disconnect();
	int init_driver();
};