#include "Levitator.h"

void printV2(const char* str) {
	printf("%s\n", str);
}

namespace microTimer {
	static DWORD uGetTime(DWORD baseTime = 0) {
		LARGE_INTEGER nFreq, currentTime;
		DWORD dwTime;

		QueryPerformanceFrequency(&nFreq);
		QueryPerformanceCounter(&currentTime);
		dwTime = (DWORD)(currentTime.QuadPart * 1000000 / nFreq.QuadPart);

		return dwTime - baseTime;
	}

	static void uWait(DWORD waitTime) {
		DWORD currentTime = uGetTime();
		while (waitTime > uGetTime(currentTime)) { ; }
	}

	static void keepUpdatePeriod(DWORD updatePeriod) {
		static DWORD prevTime = 0;
		DWORD currentTime = microTimer::uGetTime();
		while (currentTime - prevTime < updatePeriod) {
			currentTime = microTimer::uGetTime();
		}
		prevTime = currentTime;
	}
};


Levitator::Levitator(int* boardIDsIn, float* matBoardToWorldIn, int numBoardsIn, bool printIn, int update_rate_in) {
	if (print) { printf("Connecting to board..."); };
	numBoards = numBoardsIn;
	boardIDs = (int*)malloc(sizeof(int) * numBoards);
	memcpy(boardIDs, boardIDsIn, sizeof(int) * numBoards);
	matBoardToWorld = (float*)malloc(16 * sizeof(float) * numBoards);
	memcpy(matBoardToWorld, matBoardToWorldIn, 16 * sizeof(int) * numBoards);
	numTransducers = numBoards * 256;

	print = printIn;
	update_rate = update_rate_in;

	if (print) { printf("Connected to Board\n"); };

}

int Levitator::init_driver() {
	AsierInho_V2::RegisterPrintFuncs(printV2, printV2, printV2);
	AsierInho_V2::AsierInhoBoard_V2* driver = AsierInho_V2::createAsierInho();
	this->driver = driver;

	if (!driver->connect(numBoards, boardIDs, matBoardToWorld))
		printf("Failed to connect to board. \n");
	//Read parameters to be used for the solver
	transducerPositions = new float[numTransducers * 3];
	transducerNormals = new float[numTransducers * 3];
	amplitudeAdjust = new float[numTransducers];
	mappings = new int[numTransducers];
	phaseDelays = new int[numTransducers];

	driver->readParameters(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);


	disc = AsierInho::createAsierInho(); 
	disc->connect(AsierInho::BensDesign, boardIDs[0], boardIDs[1]);
	disc->disconnect();

	return 0;
}

int Levitator::sendNewDivider(unsigned int newDivider) {

	unsigned char* dividerMessage = new unsigned char[512 * numBoards];
	memset(dividerMessage, 0, 512 * numBoards * sizeof(unsigned char));

	for (int b = 0; b < numBoards; b++) {
		int divider = newDivider;
		dividerMessage[512 * b + 0] = 128;
		
		for (int bit = 0; bit < 8; bit++) {
			dividerMessage[512 * b + 26 + bit] = 128 * (divider % 2);
			divider /= 2;
		}
		dividerMessage[512 * b + 34] = 128;
	}

	driver->updateMessage(dividerMessage);
	delete[] dividerMessage;
	return 0;

}


int Levitator::sendMessages(float* phases, float* amplitudes, float relative_amp, int num_geometriesIn, int sleep_ms, bool loop) {
	num_geometries = num_geometriesIn;
	unsigned char phases_disc[512], amplitudes_disc[512];

	int numUpdateGeometries = 32;
	if (num_geometries < numUpdateGeometries) {
		numUpdateGeometries = num_geometries;
	}

	// See Bk2. Pg 70
	int number_of_packages = num_geometries / numUpdateGeometries;
	if (number_of_packages * numUpdateGeometries < num_geometries) {
		number_of_packages++;
	}

	unsigned char* messages = new unsigned char[2 * numUpdateGeometries * number_of_packages * numTransducers];

	int geometry = 0;
	for (int p = 0; p < number_of_packages; p++) { //iterate over the packages to send at once
		for (int g = 0; g < numUpdateGeometries; g++) { //
			if (geometry < num_geometries) { // a little sloppy but stops it overshooting
				disc->discretizePhases(&(phases[geometry * numTransducers]), phases_disc);
				if (amplitudes) {
					disc->discretizeAmplitudes(&(amplitudes[geometry * numTransducers]), amplitudes_disc);
					disc->correctPhasesShift(phases_disc, amplitudes_disc);
				}
				else {
					unsigned char disc_amp = disc->_discretizeAmplitude(relative_amp);
					memset(amplitudes_disc, disc_amp, numTransducers * sizeof(unsigned char));
				}
			}
				/*printf("%d %d %d %p %p %p %p\n",
					geometry,
					p,
					g,
					&messages[p * 2 * numUpdateGeometries * numTransducers + numTransducers * g],
					&messages[p * 2 * numUpdateGeometries * numTransducers + numTransducers * g + (numTransducers / 2)],
					&messages[p * 2 * numUpdateGeometries * numTransducers + numTransducers * numUpdateGeometries + numTransducers * g],
					&messages[p * 2 * numUpdateGeometries * numTransducers + numTransducers * numUpdateGeometries + numTransducers * g + (numTransducers / 2)]
				);*/
			memcpy(&messages[p * 2 * numUpdateGeometries * numTransducers + numTransducers*g], &phases_disc[0], (numTransducers / 2) * sizeof(unsigned char)); //Bottom phase 
			memcpy(&messages[p * 2 * numUpdateGeometries * numTransducers + numTransducers * g + (numTransducers / 2)], &amplitudes_disc[0], (numTransducers / 2) * sizeof(unsigned char)); //Bottom amplitude 
			messages[p * 2 * numUpdateGeometries * numTransducers + numTransducers * g] += 128; 
				
			memcpy(&messages[p * 2 * numUpdateGeometries * numTransducers + numTransducers* numUpdateGeometries + numTransducers * g], &phases_disc[numTransducers / 2], (numTransducers / 2)  * sizeof(unsigned char)); //Top phase
			memcpy(&messages[p * 2 * numUpdateGeometries * numTransducers + numTransducers * numUpdateGeometries + numTransducers * g + (numTransducers / 2)], &amplitudes_disc[numTransducers /  2], (numTransducers / 2) * sizeof(unsigned char));//Top amplitude
			messages[p * 2 * numUpdateGeometries * numTransducers + numTransducers * numUpdateGeometries + numTransducers * g] += 128; 
			
			geometry++; 
			
		}

	}

	int div = 40000 / update_rate;
	this->sendNewDivider(div);

	for (int i = 0; i < 16; i++) {
		driver->updateMessage(&messages[0]);
	}

	
	DWORD waitingPeriod = numUpdateGeometries * (1000000 / update_rate);
	DWORD lastUpdate = microTimer::uGetTime();
	DWORD currentTime = lastUpdate;
	DWORD start = microTimer::uGetTime();

	bool in_loop = false;
	while( loop || !in_loop){
		for (int g = 0; g < num_geometries; g += numUpdateGeometries) {
			// wait a while 
			do {
				currentTime = microTimer::uGetTime();
			} while (currentTime - lastUpdate < waitingPeriod);
			// send messages to the boards
			int numMessagesToSend = min(numUpdateGeometries, num_geometries - g);
			driver->updateMessages(&messages[2 * g * numTransducers], numMessagesToSend);
			// get the current time
			lastUpdate = microTimer::uGetTime();
		}
		in_loop = true;
	}
	DWORD end = microTimer::uGetTime();
	if (print) { printf("Estimated frame rate is %f Hz\n", num_geometries * 1000000.f / (end - start)); }

	return 0;
}

int Levitator::TurnOff() {
	driver->turnTransducersOff();
	return 0;
}

int Levitator::Disconnect() {
	driver->turnTransducersOff();
	Sleep(100);
	driver->disconnect();
	delete driver;
	return 0;
}

extern "C" {
	__declspec(dllexport) void* connect_to_levitator(int* boardIDsIn, float* matBoardToWorldIn, int numBoardsIn, bool print) { 
		Levitator* lev = new Levitator(boardIDsIn, matBoardToWorldIn, numBoardsIn, print);
		lev->init_driver();
		return lev;
		//return new Levitator;
	}

	__declspec(dllexport) int send_message (void* levitator_ptr, float* phases, float* amplitudes, float relative_amp, int num_geometries, int sleep_ms, bool loop) {
		try
		{
			Levitator* leviator = reinterpret_cast<Levitator*>(levitator_ptr);
			return leviator->sendMessages(phases, amplitudes, relative_amp, num_geometries,sleep_ms,loop);
		}
		catch (...)
		{
			return -1; //assuming -1 is an error condition. 
		}
	}

	__declspec(dllexport) int disconnect(void* levitator_ptr) {
		{
			try
			{
				Levitator* leviator = reinterpret_cast<Levitator*>(levitator_ptr);
				return leviator->Disconnect();
			}
			catch (...)
			{
				return -1; //assuming -1 is an error condition. 
			}
		}
	}

	__declspec(dllexport) int turn_off(void* levitator_ptr) {
		{
			try
			{
				Levitator* leviator = reinterpret_cast<Levitator*>(levitator_ptr);
				return leviator->TurnOff();
			}
			catch (...)
			{
				return -1; //assuming -1 is an error condition. 
			}
		}
	}
	

}