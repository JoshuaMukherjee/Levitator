#include "Levitator.h"

void printV2(const char* str) {
	printf("%s\n", str);
}

Levitator::Levitator(int* boardIDsIn, float* matBoardToWorldIn, int numBoardsIn, bool printIn) {
	if (print) { printf("Connecting to board..."); };
	numBoards = numBoardsIn;
	boardIDs = (int*)malloc(sizeof(int) * numBoards);
	memcpy(boardIDs, boardIDsIn, sizeof(int) * numBoards);
	matBoardToWorld = (float*)malloc(16 * sizeof(float) * numBoards);
	memcpy(matBoardToWorld, matBoardToWorldIn, 16 * sizeof(int) * numBoards);
	numTransducers = numBoards * 256;

	print = printIn;

	if (print) { printf("Connected\n"); };

}

int Levitator::init_driver() {
	AsierInho_V2::RegisterPrintFuncs(printV2, printV2, printV2);
	AsierInho_V2::AsierInhoBoard_V2* driver = AsierInho_V2::createAsierInho();
	this->driver = driver;

	if (!driver->connect(numBoards, boardIDs, matBoardToWorld))
		printf("Failed to connect to board.");
	//Read parameters to be used for the solver
	transducerPositions = new float[numTransducers * 3];
	transducerNormals = new float[numTransducers * 3];
	amplitudeAdjust = new float[numTransducers];
	mappings = new int[numTransducers];
	phaseDelays = new int[numTransducers];

	driver->readParameters(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);


	disc = AsierInho::createAsierInho(); //MOVE THIS TO INIT
	disc->connect(AsierInho::BensDesign, boardIDs[0], boardIDs[1]);
	disc->disconnect();

	return 0;
}

int Levitator::setPhaseAmplitude(float* phases, float* amplitudes, float relative_amp) {

	

	

	disc->discretizePhases(phases, phases_disc);

	if (amplitudes) {
		disc->discretizeAmplitudes(amplitudes, amplitudes_disc);
		disc->correctPhasesShift(phases_disc, amplitudes_disc);
	}
	else {

		unsigned char disc_amp = disc->_discretizeAmplitude(relative_amp);
		memset(amplitudes_disc, disc_amp, numTransducers * sizeof(unsigned char));
	}
	return 0;
}

int Levitator::sendMessage() {
	unsigned char* messages = new unsigned char[2 * numTransducers];


	memcpy(&messages[0], &phases_disc[0], (numTransducers / 2) * sizeof(unsigned char));
	memcpy(&messages[0 + numTransducers / 2], &amplitudes_disc[0], (numTransducers / 2) * sizeof(unsigned char));
	messages[0] += 128;
	
	memcpy(&messages[numTransducers], &phases_disc[numTransducers / 2], (numTransducers / 2) * sizeof(unsigned char));
	memcpy(&messages[ 3 * numTransducers / 2], &amplitudes_disc[numTransducers / 2], (numTransducers / 2) * sizeof(unsigned char));
	messages[numTransducers] += 128;

	for (int i = 0; i < 16; i++) {
		driver->updateMessage(&messages[0]);
	}

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

	__declspec(dllexport) int set_phase_amplitude(void* levitator_ptr, float* phases, float* amplitudes, float relative_amp) {
		try
		{
			Levitator* leviator = reinterpret_cast<Levitator*>(levitator_ptr);
			return leviator->setPhaseAmplitude(phases, amplitudes, relative_amp);
		}
		catch (...)
		{
			return -1; //assuming -1 is an error condition. 
		}
	}

	__declspec(dllexport) int send_message (void* levitator_ptr) {
		try
		{
			Levitator* leviator = reinterpret_cast<Levitator*>(levitator_ptr);
			return leviator->sendMessage();
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