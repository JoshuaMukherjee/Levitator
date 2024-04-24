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

	unsigned char phases_disc[512];
	unsigned char amplitudes_disc[512];


public:
	AsierInho_V2::AsierInhoBoard_V2* driver;
	AsierInho::AsierInhoBoard* disc;
	Levitator( int* boardIDsIn, float* matBoardToWorldIn, int numBoardsIn = 2,bool printIn=true);
	//Levitator();
	int setPhaseAmplitude(float* phases, float* amplitudes = NULL, float relative_amp = 1.0);
	int sendMessage();
	int TurnOff();
	int Disconnect();
	int init_driver();
};