#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <conio.h>
#include <iostream>
#include <random>


using namespace std;

std::default_random_engine generator;
std::uniform_real_distribution<double> distribution(-1, 1); //doubles from -1 to 1

int  NoVertices, NoStations, NoCustomers, NoVehicles, NoChargers, No,
* SortI, * SortII, * Seed, ** Route, ** BestRoute, ** CurrentRoute, ** InitialRoute, RoutedCustomers, 
* Technology, * Counter, * BestCounter, * CurrentCounter, * InitialCounter, * CounterStation, 
* BestCounterStation,* CurrentCounterStation, * InitialCounterStation, CounterRemoval, 
SelectionNode1, SelectionNode2,* RemovedCustomer, * RemovalStation,* RemovalRoute, NoRoutes, 
BestNoRoutes, CurrentNoRoutes,InitialNoRoutes, NoRemovalC, * Score, * NoUse, NCR, NSR, NRR, NCI, NSI,
NoOperators, IterC, IterS, EnergyLimit, TimeWindowCheck, Iter, IterO,NoEV, NoCV, CurrentNoEV, 
CurrentNoCV, BestNoEV, BestNoCV, InitialNoEV, InitialNoCV, NoSolution, ParetoNoEV[10], ParetoNoCV[10], ParetoNoChargers[10];

float *Xcoordinate, * Ycoordinate, * fCost, * RCost, * TCost, * CurrentTCost, * BestTCost, * InitialTCost, 
* MaxCapacity,* eTime, * lTime, * ServiceTime, * Rate, ALNSStartTime, ALNSEndTime, * Demand, 
MaxDistance,* MaxEnergyCapacity, * CurrentMaxEnergyCapacity, * BestMaxEnergyCapacity, 
* InitialMaxEnergyCapacity,* EnergyConsumption, * BestEnergyConsumption, * CurrentEnergyConsumption, 
* InitialEnergyConsumption,* AcquisitionCost, * CurrentAcquisitionCost, * BestAcquisitionCost, 
* InitialAcquisitionCost, Cost, BestCost, CurrentCost, InitialCost, * RouteLength, 
* BestRouteLength, * CurrentRouteLength, * InitialRouteLength, Emissions, BestEmissions, 
CurrentEmissions, InitialEmissions,** Weight, ** Probability, * Pr, RouletteWheelPar, BigNumber, 
Epsilon, Ratio, *** vPtr, ** qPtr, ** yPtr, ** uPtr, *** TempV, ** TempQ, * TempU, ** TempY, 
** t, ** d,*** BestV, ** BestQ, ** BestU, ** BestY, *** CurrentV, ** CurrentQ, ** CurrentU, 
** CurrentY,*** InitialV, ** InitialQ, ** InitialU, ** InitialY, Pareto[10], FirstObj[10],
SecondObj[10], EVLength,CVLength, ParetoEVLength[10], ParetoCVLength[10], WeightedSum,
BestWeightedSum, CurrentWeightedSum;


FILE* Input, * Output;
char FileName[1500], FileNameOut[1500];
double PI = 4.0*atan(1.0);

char OutName[500];
FILE* Out;

float second() {
	return((float)clock() / (float)CLK_TCK);
}

	// Second Objective Parameters
	int CurbWeight = 15000;
	float EfficiencyParameter = 0.9;
	float DriveTrain = 0.4;

	float Rho = 1.00/(1000.00 * EfficiencyParameter * DriveTrain);
	//cout<<Rho<<"\n";

	float Acceleration = 0.68;
	float GravitationalConstant = 9.81;
	float RoadAngle = 4.57;
	float RollingResistance = 0.01;

	float Sigma2nd = Acceleration + GravitationalConstant * (sin(RoadAngle*(PI/180.00)) + (RollingResistance * cos(RoadAngle*(PI/180.00))));
	float Sin1 = sin(RoadAngle*(PI/180.00));
	float Cos1 = cos(RoadAngle*(PI/180.00));
	//cout<<Sin1<<"\n";
	//cout<<Cos1<<"\n";
	//cout<<Sigma2nd<<"\n";

	float FuelToAirMass = 1;
	float ConversionFactor = 737;
	float HeatingValue = 44;

	float Lambda = FuelToAirMass / (ConversionFactor * HeatingValue);
	//cout<<Lambda<<"\n";

	float EngineFriction = 0.2;
	int EngineSpeed = 33;
	int EngineDisplacement = 5;


	float AerodynamicDrag = 0.7;
	float AirDensity = 1.2041;
	float FrontalSurface = 7.2;

	float Epsilon2nd = 0.5 * AerodynamicDrag * AirDensity * FrontalSurface;
	//cout<<Epsilon2nd<<"\n";
float speed = 1.00; // average speed is 60 km/h or 1 km/min


void DistanceFunction() {
	int i, j, k;
	MaxDistance = 0;
	float earth = 6378.137;
	double PI = 4.0 * atan(1.0);

	for (i = 0; i <= NoVertices; i++) {
		d[i][i] = 0;
		for (j = i + 1; j <= NoVertices; j++) {
			double dlat1 = Xcoordinate[i] * (PI / 180);
			double dlong1 = Ycoordinate[i] * (PI / 180);
			double dlat2 = Xcoordinate[j] * (PI / 180);
			double dlong2 = Ycoordinate[j] * (PI / 180);

			double dLong = dlong1 - dlong2;
			double dLat = dlat1 - dlat2;

			double aHarv = pow(sin(dLat / 2.0), 2.0) + cos(dlat1) * cos(dlat2) * pow(sin(dLong / 2), 2);
			double cHarv = 2 * atan2(sqrt(aHarv), sqrt(1.0 - aHarv));

			d[i][j] = 1.5 * earth * (float) cHarv; // Real distances
			d[j][i] = d[i][j];
			t[i][j] = speed * d[i][j]; 
			t[j][i] = t[i][j];
			if (d[i][j] > MaxDistance) {
				MaxDistance = d[i][j];
			}
		}
	}
	/*for (i = 0; i <= NoVertices; i++){
		d[i][i] = 0;
		for (j = i + 1; j <= NoVertices; j++){
			d[i][j] = sqrt( (Xcoordinate[i] - Xcoordinate[j])*(Xcoordinate[i] - Xcoordinate[j]) +
							(Ycoordinate[i] - Ycoordinate[j])*(Ycoordinate[i] - Ycoordinate[j]) );

			d[j][i] = d[i][j];
			if( d[i][j] > MaxDistance ){
				MaxDistance = d[i][j];
			}
			//cout<<"d["<<i<<"]["<<j<<"]:"<<d[i][j]<<"\n";
			t[i][j] = d[i][j];
			t[j][i] = t[i][j];
		}
	}*/
}

void ReadData(int index1, int index2, int index3) {

	int i, j, k, m;
	int temp;
	NoChargers = 3;
	NCR = 8;
	NSR = 4;
	NRR = 2;
	NCI = 3;
	NSI = 3;
	NoOperators = NCR + NCI + NSR + NSI + NRR;
	IterC = 500; //??
	IterS = 1000; //??
	EnergyLimit = 1000;
	BigNumber = 999999;
	Cost = CurrentCost = BestCost = InitialCost = 0;
	Emissions = CurrentEmissions = BestEmissions = InitialEmissions = 0;
	WeightedSum = 0, BestWeightedSum = 0, CurrentWeightedSum = 0;
	NoEV = 0;
	NoCV = 0;

	fscanf_s(Input, "%f", &Ratio);
	//cout<<"Ratio = "<<Ratio<<"\n";
	fscanf_s(Input, "%d", &NoVertices);
	//cout<<"NoVertices = "<<NoVertices<<"\n";
	fscanf_s(Input, "%d", &NoStations);
	//cout<<"NoStations = "<<NoStations<<"\n";
	fscanf_s(Input, "%d", &NoCustomers);
	//cout<<"NoCustomers = "<<NoCustomers<<"\n";
	fscanf_s(Input, "%d", &NoVehicles);
	//cout<<"NoVehicles = "<<NoVehicles<<"\n";

	/// Initialize Parameters /////

	Xcoordinate = new float[NoVertices + 1];
	Ycoordinate = new float[NoVertices + 1];
	Demand = new float[NoVertices + 1];
	eTime = new float[NoVertices + 1];
	lTime = new float[NoVertices + 1];
	ServiceTime = new float[NoVertices + 1];

	fCost = new float[NoVehicles + 1];
	TCost = new float[NoVehicles + 1];
	CurrentTCost = new float[NoVehicles + 1];
	BestTCost = new float[NoVehicles + 1];
	InitialTCost = new float[NoVehicles + 1];
	MaxCapacity = new float[NoVehicles + 1];
	MaxEnergyCapacity = new float[NoVehicles + 1];
	CurrentMaxEnergyCapacity = new float[NoVehicles + 1];
	BestMaxEnergyCapacity = new float[NoVehicles + 1];
	InitialMaxEnergyCapacity = new float[NoVehicles + 1];
	EnergyConsumption = new float[NoVehicles + 1];
	BestEnergyConsumption = new float[NoVehicles + 1];
	CurrentEnergyConsumption = new float[NoVehicles + 1];
	InitialEnergyConsumption = new float[NoVehicles + 1];
	AcquisitionCost = new float[NoVehicles + 1];
	CurrentAcquisitionCost = new float[NoVehicles + 1];
	BestAcquisitionCost = new float[NoVehicles + 1];
	InitialAcquisitionCost = new float[NoVehicles + 1];

	RCost = new float[NoChargers + 1];
	Rate = new float[NoVertices + 1];


	EnergyConsumption = new float[NoVehicles + 1];
	Score = new int[NoOperators + 1];
	NoUse = new int[NoOperators + 1];
	Weight = new float* [IterS + IterC + 1];
	for (int j = 0; j <= IterS + IterC; j++)
		Weight[j] = new float[NoOperators + 1];
	Pr = new float[NoOperators + 1];
	Probability = new float* [IterS + IterC + 1];
	for (int j = 0; j <= IterS + IterC; j++)
		Probability[j] = new float[NoOperators + 1];

	///////////////// Initialize ptr-vectors /////////////////


	vPtr = new float** [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		vPtr[i] = new float* [NoChargers + 1];
		for (j = 0; j <= NoChargers; j++) {
			vPtr[i][j] = new float[NoVehicles + 1];
		}
	}

	qPtr = new float* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		qPtr[i] = new float[NoVehicles + 1];
	}

	yPtr = new float* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		yPtr[i] = new float[NoVehicles + 1];
	}
	uPtr = new float* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		uPtr[i] = new float[NoVehicles + 1];
	}

	for (i = 0; i <= NoVertices ; i++) {
		for (m = 0; m <= NoChargers; m++) {
			for (k = 0; k <= NoVehicles; k++) {
				vPtr[i][m][k] = 0;
			}
		}
	}

	for (i = 0; i <= NoVertices ; i++) {
		for (k = 0; k <= NoVehicles; k++) {
			qPtr[i][k] = 0;
			yPtr[i][k] = 0;
			uPtr[i][k] = 0;

		}
	}

		//// Programming variables ////
	SortI = new int[NoVertices + 1];
	SortII = new int[NoVertices + 1];
	//SortIII = new int[NoChargers + 1];
	Seed = new int[NoVertices + 1];
	RouteLength = new float[NoVehicles + 1];
	BestRouteLength = new float[NoVehicles + 1];
	CurrentRouteLength = new float[NoVehicles + 1];
	InitialRouteLength = new float[NoVehicles + 1];
	Route = new int* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		Route[i] = new int[NoVehicles + 1];
	}
	CurrentRoute = new int* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		CurrentRoute[i] = new int[NoVehicles + 1];
	}
	BestRoute = new int* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		BestRoute[i] = new int[NoVehicles + 1];
	}
	InitialRoute = new int* [NoVertices + 1];
	for (i = 0; i <= NoVertices; i++) {
		InitialRoute[i] = new int[NoVehicles + 1];
	}
	//Penalty = new float[NoChargers + 1];
	Counter = new int[NoVehicles + 1];
	CurrentCounter = new int[NoVehicles + 1];
	BestCounter = new int[NoVehicles + 1];
	InitialCounter = new int[NoVehicles + 1];
	CounterStation = new int[NoCustomers + NoStations + 1];
	BestCounterStation = new int[NoCustomers + NoStations + 1];
	CurrentCounterStation = new int[NoCustomers + NoStations + 1];
	InitialCounterStation = new int[NoCustomers + NoStations + 1];
	Technology = new int[NoCustomers + NoStations + 1];
	RemovedCustomer = new int[NoCustomers + 1];
	RemovalStation = new int[NoVertices + 1];
	RemovalRoute = new int[NoVehicles + 1];

	TempV = new float** [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		TempV[i] = new float* [NoChargers + 1];
		for (j = 0; j <= NoChargers; j++) {
			TempV[i][j] = new float[NoVehicles + 1];
		}
	}

	TempQ = new float* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		TempQ[i] = new float[NoVehicles + 1];
	}
	TempY = new float* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		TempY[i] = new float[NoVehicles + 1];
	}
	TempU = new float[NoVertices  + 1];

	CurrentV = new float** [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		CurrentV[i] = new float* [NoChargers + 1];
		for (j = 0; j <= NoChargers; j++) {
			CurrentV[i][j] = new float[NoVehicles + 1];
		}
	}
	CurrentQ = new float* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		CurrentQ[i] = new float[NoVehicles + 1];
	}

	CurrentY = new float* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		CurrentY[i] = new float[NoVehicles + 1];
	}
	CurrentU = new float* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		CurrentU[i] = new float[NoVehicles + 1];
	}

	BestV = new float** [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		BestV[i] = new float* [NoChargers + 1];
		for (j = 0; j <= NoChargers; j++) {
			BestV[i][j] = new float[NoVehicles + 1];
		}
	}
	BestQ = new float* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		BestQ[i] = new float[NoVehicles + 1];
	}

	BestY = new float* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		BestY[i] = new float[NoVehicles + 1];
	}
	BestU = new float* [NoVertices  + 1];
	for (i = 0; i <= NoVertices ; i++) {
		BestU[i] = new float[NoVehicles + 1];
	}
	InitialV = new float** [NoVertices + 1];
	for (i = 0; i <= NoVertices; i++) {
		InitialV[i] = new float* [NoChargers + 1];
		for (j = 0; j <= NoChargers; j++) {
			InitialV[i][j] = new float[NoVehicles + 1];
		}
	}
	InitialQ = new float* [NoVertices + 1];
	for (i = 0; i <= NoVertices; i++) {
		InitialQ[i] = new float[NoVehicles + 1];
	}

	InitialY = new float* [NoVertices + 1];
	for (i = 0; i <= NoVertices; i++) {
		InitialY[i] = new float[NoVehicles + 1];
	}
	InitialU = new float* [NoVertices + 1];
	for (i = 0; i <= NoVertices; i++) {
		InitialU[i] = new float[NoVehicles + 1];
	}
	t = new float* [NoVertices + 1];
	for (i = 0; i <= NoVertices; i++) {
		t[i] = new float[NoVertices + 1];
	}

	d = new float* [NoVertices + 1];
	for (i = 0; i <= NoVertices; i++) {
		d[i] = new float[NoVertices + 1];
	}

	for (i = 0; i <= NoVertices; i++) {
		SortI[i] = 0;
		SortII[i] = 0;
		Seed[i] = 0;
		TempU[i] = 0;
	}

	/*for (i = 0; i <= NoChargers; i++) {
		SortIII[i] = 0;
	}*/

	for (i = 0; i <= NoVehicles; i++) {
		Counter[i] = 0;
		CurrentCounter[i] = 0;
		BestCounter[i] = 0;
		InitialCounter[i] = 0;
		CurrentMaxEnergyCapacity[i] = 0;
		BestMaxEnergyCapacity[i] = 0;
		InitialMaxEnergyCapacity[i] = 0;
		CurrentEnergyConsumption[i] = 0;
		BestEnergyConsumption[i] = 0;
		InitialEnergyConsumption[i] = 0;
		CurrentTCost[i] = 0;
		BestTCost[i] = 0;
		InitialTCost[i] = 0;
		CurrentAcquisitionCost[i] = 0;
		BestAcquisitionCost[i] = 0;
		InitialAcquisitionCost[i] = 0;
		RouteLength[i] = 0;
		CurrentRouteLength[i] = 0;
		BestRouteLength[i] = 0;
		InitialRouteLength[i] = 0;
	}
	for (i = 0; i <= NoCustomers + NoStations; i++) {
		CounterStation[i] = 0;
		CurrentCounterStation[i] = 0;
		BestCounterStation[i] = 0;
		InitialCounterStation[i] = 0;
	}
	for (i = 0; i <= NoVertices ; i++) {
		for (k = 0; k <= NoVehicles; k++) {
			Route[i][k] = 0;
			CurrentRoute[i][k] = 0;
			BestRoute[i][k] = 0;
			InitialRoute[i][k] = 0;

		}
	}
	/*for (m = 0; m <= NoChargers; m++) {
		Penalty[m] = 0;
	}*/
	for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
		Technology[i] = 0;
	}
	for (i = 0; i <= NoCustomers; i++) {
		RemovedCustomer[i] = 0;
	}
	for (i = 0; i <= NoVertices; i++) {
		RemovalStation[i] = 0;
	}
	for (i = 0; i <= NoVehicles; i++) {
		RemovalRoute[i] = 0;
	}
	for (i = 0; i <= NoVertices ; i++) {
		for (m = 0; m <= NoChargers; m++) {
			for (k = 0; k <= NoVehicles; k++) {
				TempV[i][m][k] = 0;
			}
		}
	}

	for (i = 0; i <= NoVertices ; i++) {
		for (k = 0; k <= NoVehicles; k++) {
			TempQ[i][k] = 0;
			TempY[i][k] = 0;
			CurrentQ[i][k] = 0;
			CurrentY[i][k] = 0;
			CurrentU[i][k] = 0;
			BestQ[i][k] = 0;
			BestY[i][k] = 0;
			BestU[i][k] = 0;
			InitialQ[i][k] = 0;
			InitialY[i][k] = 0;
			InitialU[i][k] = 0;

		}
	}

	for (i = 0; i <= NoVertices ; i++) {
		for (m = 0; m <= NoChargers; m++) {
			for (k = 0; k <= NoVehicles; k++) {
				CurrentV[i][m][k] = 0;
				BestV[i][m][k] = 0;
				InitialV[i][m][k] = 0;
			}
		}
	}

	for (i = 0; i <= NoOperators; i++) {
		Score[i] = 0;
		NoUse[i] = 0;
		Pr[i] = 0;
	}
	for (i = 0; i <= IterS + IterC; i++) {
		for (j = 0; j <= NoOperators; j++) {
			Weight[i][j] = 1.00;
			Probability[i][j] = 0.00;
		}
	}

	for (i = 0; i <= NoVertices; i++) {
		fscanf_s(Input, "%d", &temp);
		//cout<<"temp = "<<temp<<"\n";
		fscanf_s(Input, "%f", &Xcoordinate[i]);
		//cout<<"X["<<i<<"] = "<<Xcoordinate[i]<<"\n";
		fscanf_s(Input, "%f", &Ycoordinate[i]);
		//cout<<"Y["<<i<<"] = "<<Ycoordinate[i]<<"\n";
		fscanf_s(Input, "%f", &Demand[i]);
		//cout<<"Demand["<<i<<"] = "<<Demand[i]<<"\n";
		fscanf_s(Input, "%f", &eTime[i]);
		//cout<<"eTime["<<i<<"] = "<<eTime[i]<<"\n";
		fscanf_s(Input, "%f", &lTime[i]);
		//cout<<"lTime["<<i<<"] = "<<lTime[i]<<"\n";
		fscanf_s(Input, "%f", &ServiceTime[i]);
		//cout<<"ServiceTime["<<i<<"] = "<<ServiceTime[i]<<"\n";
	}

	for (j = 1; j <= NoVehicles; j++) {
		fscanf_s(Input, "%d", &temp);
		//cout<<"temp = "<<temp<<"\n";
		fscanf_s(Input, "%f", &TCost[j]);
		//cout<<"TCost["<<j<<"] = "<<TCost[j]<<"\n";
		fscanf_s(Input, "%f", &MaxCapacity[j]);
		//cout<<"MaxCapacity["<<j<<"] = "<<MaxCapacity[j]<<"\n";
		fscanf_s(Input, "%f", &MaxEnergyCapacity[j]);
		//cout<<"MaxEnergyCapacity["<<j<<"] = "<<MaxEnergyCapacity[j]<<"\n";
		fscanf_s(Input, "%f", &EnergyConsumption[j]);
		//cout<<"EnergyConsumption["<<j<<"] = "<<EnergyConsumption[j]<<"\n";
		fscanf_s(Input, "%f", &AcquisitionCost[j]);
		//cout<<"EnergyConsumption["<<j<<"] = "<<AcquisitionCost[j]<<"\n";
	}

	for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
		fscanf_s(Input, "%d", &temp);
		//cout<<"temp = "<<temp<<"\n";
		fscanf_s(Input, "%f", &Rate[i]);
		//cout<<"Rate["<<i<<"]["<<j<<"] = "<<Rate[i][j]<<"\n";
	}

	for (j = 1; j <= NoChargers; j++) {
		fscanf_s(Input, "%d", &temp);
		//cout<<"temp = "<<temp<<"\n";
		fscanf_s(Input, "%f", &RCost[j]);
		//cout<<"RCost["<<j<<"] = "<<RCost[j]<<"\n";
	}

	for (i = 0; i <= NoVertices; i++) {
		for (j = 0; j <= NoVertices; j++) {
			d[i][j] = 0;
			//printf("%3d\n", d[i][j]);
		}
	}

	fclose(Input);
	DistanceFunction();
}

int UpdateTimeI(int Variable1, int Variable2) {
	//Update Temporary Arrival Time for the last node after the selected node //Yes
	if (eTime[Variable1] > TempU[Variable1]) {
		TempU[Variable2] = eTime[Variable1] + ServiceTime[Variable1] + t[Variable1][Variable2];
	}
	else {
		TempU[Variable2] = TempU[Variable1] + ServiceTime[Variable1] + t[Variable1][Variable2];
	}
	return TempU[Variable2];
}

int UpdateTimeII(int Variable1, int Variable2) {
	//Update Temporary Arrival Time for the selected node  Yes
	if (eTime[Route[Counter[Variable1] - 1][Variable1]] <= uPtr[Counter[Variable1] - 1][Variable1]) {
		if (Route[Counter[Variable1] - 1][Variable1] > NoCustomers) { ////  Check if the previous node is a station or a customer 
			if (Technology[Route[Counter[Variable1] - 1][Variable1]] > 0 && Technology[Route[Counter[Variable1] - 1][Variable1]] <= 3) {    //  Update Time for the station Yes
				TempU[Variable2] = uPtr[Counter[Variable1] - 1][Variable1] + vPtr[Counter[Variable1] - 1][Technology[Route[Counter[Variable1] - 1][Variable1]]][Variable1] * Rate[Route[Counter[Variable1] - 1][Variable1]] + ServiceTime[Route[Counter[Variable1] - 1][Variable1]] + t[Route[Counter[Variable1] - 1][Variable1]][Variable2];
			}
		}
		else {
			TempU[Variable2] = uPtr[Counter[Variable1] - 1][Variable1] + ServiceTime[Route[Counter[Variable1] - 1][Variable1]] + t[Route[Counter[Variable1] - 1][Variable1]][Variable2];
		}
	}
	else {
		if (Route[Counter[Variable1] - 1][Variable1] > NoCustomers) { ////  Check if the previous node is a station or a customer Yes
			if (Technology[Route[Counter[Variable1] - 1][Variable1]] > 0 && Technology[Route[Counter[Variable1] - 1][Variable1]] <= 3) {    //  Update Time for the station ??
				TempU[Variable2] = eTime[Route[Counter[Variable1] - 1][Variable1]] + vPtr[Counter[Variable1] - 1][Technology[Route[Counter[Variable1] - 1][Variable1]]][Variable1] * Rate[Route[Counter[Variable1] - 1][Variable1]] + ServiceTime[Route[Counter[Variable1] - 1][Variable1]] + t[Route[Counter[Variable1] - 1][Variable1]][Variable2];
			}
		}
		else {
			TempU[Variable2] = eTime[Route[Counter[Variable1] - 1][Variable1]] + ServiceTime[Route[Counter[Variable1] - 1][Variable1]] + t[Route[Counter[Variable1] - 1][Variable1]][Variable2];
		}
	}
	return TempU[Variable2];
}

int SortStations(int Variable1, int Variable2) {
	int i, j, l;
	for (i = NoCustomers + 1; i <= NoCustomers + NoStations - 1; i++) { // Yes
		for (j = i + 1; j <= NoCustomers + NoStations; j++) {
			if ((d[Variable1][SortII[i]] + d[SortII[i]][Variable2]) >= (d[Variable1][SortII[j]] + d[SortII[j]][Variable2])) {
				int temp;
				temp = SortII[j];
				for (l = j - 1; l >= i; l--) {
					SortII[l + 1] = SortII[l];
					//cout<<SortII[l+1]<<endl;
				}
				SortII[i] = temp;
				//cout<<SortII[i]<<endl;
			}
		}
	}
	if (SortII[NoCustomers + 1] == Variable1 || SortII[NoCustomers + 1] == Variable2) {
		int temp = SortII[NoCustomers + 1];
		for (j = NoCustomers + 1; j <= NoCustomers + NoStations; j++) {
			SortII[j] = SortII[j + 1];
		}
		SortII[NoCustomers + NoStations] = temp;
	}
	return SortII[NoCustomers + 1];
}

void TechnologySelectionI(int Variable1, int Variable2, int Variable3) {

	TempY[Variable1][Variable2] = yPtr[Counter[Variable2]][Variable2] - EnergyConsumption[Variable2] * d[Route[Counter[Variable2]][Variable2]][Variable1];
	if (TempY[Variable1][Variable2] > 0) {
		TempV[Variable1][Technology[Variable1]][Variable2] = 0.8 * MaxEnergyCapacity[Variable2] - TempY[Variable1][Variable2];
	}
	else {
		TempV[Variable1][Technology[Variable1]][Variable2] = 0.8 * MaxEnergyCapacity[Variable2];
	}
	TempY[Variable3][Variable2] = TempY[Variable1][Variable2] + TempV[Variable1][Technology[Variable1]][Variable2] - EnergyConsumption[Variable2] * d[Variable1][Variable3]; // We assume that the vehicle has enough energy to travel from each station to the end depot
	TempU[Variable1] = uPtr[Counter[Variable2]][Variable2] + ServiceTime[Route[Counter[Variable2]][Variable2]] + t[Route[Counter[Variable2]][Variable2]][Variable1];
	TempU[Variable3] = TempU[Variable1] + TempV[Variable1][Technology[Variable1]][Variable2] * Rate[Variable1] + ServiceTime[Variable1] + t[Variable1][Variable3];
}

void TechnologySelectionII(int Variable1, int Variable2, int Variable3) {
	if (TempY[Variable1][Variable3] < EnergyLimit) {
		if (TempY[Variable1][Variable3] > 0) {
			TempV[Variable1][Technology[Variable1]][Variable3] = 0.8 * MaxEnergyCapacity[Variable3] - TempY[Variable1][Variable3];
		}
		else {
			TempV[Variable1][Technology[Variable1]][Variable3] = 0.8 * MaxEnergyCapacity[Variable3];
		}
	}
	TempY[Variable2][Variable3] = TempY[Variable1][Variable3] + TempV[Variable1][Technology[Variable1]][Variable3] - EnergyConsumption[Variable3] * d[Variable1][Variable2];
	TempU[Variable2] = TempU[Variable1] + TempV[Variable1][Technology[Variable1]][Variable3] * Rate[Variable1] + ServiceTime[Variable1] + t[Variable1][Variable2];
}

void UpdateEnergyI(int Variable1, int Variable2) {
	if (Route[Counter[Variable1] - 1][Variable1] > NoCustomers) {  ////  Check if the previous node is a station or a customer Yes
		if (Technology[Route[Counter[Variable1] - 1][Variable1]] > 0 && Technology[Route[Counter[Variable1] - 1][Variable1]] <= 3) {  //  Update Energy for the selected node  Yes
			TempY[Variable2][Variable1] = yPtr[Counter[Variable1] - 1][Variable1] + vPtr[Counter[Variable1] - 1][Technology[Route[Counter[Variable1] - 1][Variable1]]][Variable1] - EnergyConsumption[Variable1] * d[Route[Counter[Variable1] - 1][Variable1]][Variable2];
		}
	}
	else {
		TempY[Variable2][Variable1] = yPtr[Counter[Variable1] - 1][Variable1] - EnergyConsumption[Variable1] * d[Route[Counter[Variable1] - 1][Variable1]][Variable2];
	}
}

void RouteConstruction(int k) {
	float TempValue;
	int i, j, FirstStation;
	for (j = 1; j <= NoCustomers; j++) {
		if (Seed[SortI[j]] == 0) {
			TempQ[SortI[j]][k] = qPtr[Counter[k] - 1][k] - Demand[Route[Counter[k] - 1][k]];  //  Update Load for the selected node
			if (TempQ[SortI[j]][k] >= Demand[SortI[j]]) {  /// Check for load capacity
				UpdateTimeII(k, SortI[j]);
				UpdateTimeI(SortI[j], Route[Counter[k]][k]);
				if (TempU[SortI[j]] <= lTime[SortI[j]] && TempU[Route[Counter[k]][k]] <= lTime[Route[Counter[k]][k]]) {  // Check Time Window for the selected node and the end depot
					UpdateEnergyI(k, SortI[j]);
					if (TempY[SortI[j]][k] > EnergyConsumption[k] * d[SortI[j]][Route[Counter[k]][k]]) {   /// Check if the vehicle has enough energy to travel to the last node 
						// Direct - Direct   //Yes
						//Add selected node to the end before the end depot
						RouteLength[k] = RouteLength[k] - d[Route[Counter[k] - 1][k]][Route[Counter[k]][k]] + d[Route[Counter[k] - 1][k]][SortI[j]] + d[SortI[j]][Route[Counter[k]][k]];
						Route[Counter[k] + 1][k] = Route[Counter[k]][k];
						Route[Counter[k]][k] = SortI[j];
						qPtr[Counter[k]][k] = TempQ[SortI[j]][k];
						uPtr[Counter[k]][k] = TempU[SortI[j]];
						yPtr[Counter[k]][k] = TempY[SortI[j]][k];
						qPtr[Counter[k] + 1][k] = qPtr[Counter[k]][k] - Demand[Route[Counter[k]][k]];
						uPtr[Counter[k] + 1][k] = TempU[Route[Counter[k] + 1][k]];
						yPtr[Counter[k] + 1][k] = yPtr[Counter[k]][k] - EnergyConsumption[k] * d[Route[Counter[k]][k]][Route[Counter[k] + 1][k]];
						Seed[SortI[j]] = 1;
						Counter[k] ++;
						RoutedCustomers++;
					} //// The vehicle does not have enough energy to travel to the last node
					else {
						if (TempY[SortI[j]][k] < 0) { //Check if the vehicle has not enough energy to reach the selected node
						Label2:
							//Indirect - Direct or Indirect - Indirect  Yes
							SortStations(Route[Counter[k] - 1][k], SortI[j]);
							UpdateTimeII(k, SortII[NoCustomers + 1]);
							if (TempU[SortII[NoCustomers + 1]] <= lTime[SortII[NoCustomers + 1]]) {  //Check Time Window for the station
								TempQ[SortII[NoCustomers + 1]][k] = qPtr[Counter[k] - 1][k] - Demand[Route[Counter[k] - 1][k]];
								UpdateEnergyI(k, SortII[NoCustomers + 1]);
								if (TempY[SortII[NoCustomers + 1]][k] > EnergyConsumption[k] * d[SortII[NoCustomers + 1]][SortI[j]]) {
									TechnologySelectionII(SortII[NoCustomers + 1], SortI[j], k);
									TempQ[SortI[j]][k] = TempQ[SortII[NoCustomers + 1]][k] - Demand[SortII[NoCustomers + 1]];
									UpdateTimeI(SortI[j], Route[Counter[k]][k]);
									if (TempU[Route[Counter[k]][k]] <= lTime[Route[Counter[k]][k]]) { //Check Time Window for the last node
										if (TempY[SortI[j]][k] > EnergyConsumption[k] * d[SortI[j]][Route[Counter[k]][k]]) { //Check if the vehicle can travel to the end depot from the selected node
											//Indirect - Direct  //Yes
											RouteLength[k] = RouteLength[k] - d[Route[Counter[k] - 1][k]][Route[Counter[k]][k]] + d[Route[Counter[k] - 1][k]][SortII[NoCustomers + 1]] + d[SortII[NoCustomers + 1]][Route[Counter[k]][k]];
											RouteLength[k] = RouteLength[k] - d[SortII[NoCustomers + 1]][Route[Counter[k]][k]] + d[SortII[NoCustomers + 1]][SortI[j]] + d[SortI[j]][Route[Counter[k]][k]];
											Route[Counter[k] + 2][k] = Route[Counter[k]][k];
											uPtr[Counter[k] + 2][k] = TempU[Route[Counter[k]][k]];
											Route[Counter[k] + 1][k] = SortI[j];
											Route[Counter[k]][k] = SortII[NoCustomers + 1];
											for (i = Counter[k]; i <= Counter[k] + 1; i++) {
												yPtr[i][k] = TempY[Route[i][k]][k];
												uPtr[i][k] = TempU[Route[i][k]];
												qPtr[i][k] = TempQ[Route[i][k]][k];
											}
											vPtr[Counter[k]][Technology[SortII[NoCustomers + 1]]][k] = TempV[Route[Counter[k]][k]][Technology[SortII[NoCustomers + 1]]][k];
											yPtr[Counter[k] + 2][k] = yPtr[Counter[k] + 1][k] - EnergyConsumption[k] * d[Route[Counter[k] + 1][k]][Route[Counter[k] + 2][k]];
											qPtr[Counter[k] + 2][k] = qPtr[Counter[k] + 1][k] - Demand[Route[Counter[k] + 1][k]];
											Counter[k] += 2;
											CounterStation[SortII[NoCustomers + 1]] ++;
											Seed[SortI[j]] = 1;
											RoutedCustomers++;
										}
										else {
											FirstStation = SortII[NoCustomers + 1];
											//Indirect - Indirect ???
											SortStations(SortI[j], Route[Counter[k]][k]);
											UpdateTimeI(SortI[j], SortII[NoCustomers + 1]);
											if (TempU[SortII[NoCustomers + 1]] <= lTime[SortII[NoCustomers + 1]]) {  //Check Time Window for the station
												TempQ[SortII[NoCustomers + 1]][k] = TempQ[SortI[j]][k] - Demand[SortI[j]];
												TempY[SortII[NoCustomers + 1]][k] = TempY[SortI[j]][k] - EnergyConsumption[k] * d[SortI[j]][SortII[NoCustomers + 1]];
												if (FirstStation == SortII[NoCustomers + 1]) {
													TempValue = TempY[SortII[NoCustomers + 1]][k];
												}
												TechnologySelectionII(SortII[NoCustomers + 1], Route[Counter[k]][k], k);
												if (TempY[SortII[NoCustomers + 1]][k] >= EnergyConsumption[k] * d[SortII[NoCustomers + 1]][Route[Counter[k]][k]]) {
													if (TempY[Route[Counter[k]][k]] >= 0) {
														if (TempU[Route[Counter[k]][k]] <= lTime[Route[Counter[k]][k]]) {  // Check Time Window for the last node
															RouteLength[k] = RouteLength[k] - d[Route[Counter[k] - 1][k]][Route[Counter[k]][k]] + d[Route[Counter[k] - 1][k]][SortII[NoCustomers + 1]] + d[SortII[NoCustomers + 1]][Route[Counter[k]][k]];
															RouteLength[k] = RouteLength[k] - d[SortII[NoCustomers + 1]][Route[Counter[k]][k]] + d[SortII[NoCustomers + 1]][SortI[j]] + d[SortI[j]][Route[Counter[k]][k]];
															RouteLength[k] = RouteLength[k] - d[SortI[j]][Route[Counter[k]][k]] + d[SortI[j]][FirstStation] + d[FirstStation][Route[Counter[k]][k]];
															Route[Counter[k] + 3][k] = Route[Counter[k]][k];
															yPtr[Counter[k] + 3][k] = TempY[Route[Counter[k]][k]][k];
															uPtr[Counter[k] + 3][k] = TempU[Route[Counter[k]][k]];
															Route[Counter[k] + 2][k] = SortII[NoCustomers + 1];
															Route[Counter[k] + 1][k] = SortI[j];
															Route[Counter[k]][k] = FirstStation;
															for (i = Counter[k]; i <= Counter[k] + 2; i++) {
																yPtr[i][k] = TempY[Route[i][k]][k];
																uPtr[i][k] = TempU[Route[i][k]];
																qPtr[i][k] = TempQ[Route[i][k]][k];
															}
															if (FirstStation == SortII[NoCustomers + 1]) {
																yPtr[Counter[k]][k] = TempValue;
															}
															vPtr[Counter[k] + 2][Technology[SortII[NoCustomers + 1]]][k] = TempV[Route[Counter[k] + 2][k]][Technology[SortII[NoCustomers + 1]]][k];
															vPtr[Counter[k]][Technology[FirstStation]][k] = TempV[Route[Counter[k]][k]][Technology[FirstStation]][k];
															qPtr[Counter[k] + 3][k] = qPtr[Counter[k] + 2][k] - Demand[Route[Counter[k] + 2][k]];
															Counter[k] += 3;
															CounterStation[SortII[NoCustomers + 1]] += 2;
															Seed[SortI[j]] = 1;
															RoutedCustomers++;
														}
													}
												}
											}
										}
									}
								}
							}
						}
						else {
							//Direct - Indirect    //Yes
							SortStations(SortI[j], Route[Counter[k]][k]);
							if (TempY[SortI[j]][k] >= EnergyConsumption[k] * d[SortI[j]][SortII[NoCustomers + 1]]) { //Check if the vehicle has enough energy to reach the station 
								UpdateTimeI(SortI[j], SortII[NoCustomers + 1]);
								if (TempU[SortII[NoCustomers + 1]] <= lTime[SortII[NoCustomers + 1]]) {  //Check Time Window for the station
									TempQ[SortII[NoCustomers + 1]][k] = TempQ[SortI[j]][k] - Demand[SortI[j]];
									TempY[SortII[NoCustomers + 1]][k] = TempY[SortI[j]][k] - EnergyConsumption[k] * d[SortI[j]][SortII[NoCustomers + 1]];
									TechnologySelectionII(SortII[NoCustomers + 1], Route[Counter[k]][k], k);
									if (TempU[Route[Counter[k]][k]] <= lTime[Route[Counter[k]][k]]) {  // Check Time Window for the last node
										RouteLength[k] = RouteLength[k] - d[Route[Counter[k] - 1][k]][Route[Counter[k]][k]] + d[Route[Counter[k] - 1][k]][SortI[j]] + d[SortI[j]][Route[Counter[k]][k]];
										RouteLength[k] = RouteLength[k] - d[SortI[j]][Route[Counter[k]][k]] + d[SortI[j]][SortII[NoCustomers + 1]] + d[SortII[NoCustomers + 1]][Route[Counter[k]][k]];
										Route[Counter[k] + 2][k] = Route[Counter[k]][k];
										uPtr[Counter[k] + 2][k] = TempU[Route[Counter[k]][k]];
										yPtr[Counter[k] + 2][k] = TempY[Route[Counter[k]][k]][k];
										Route[Counter[k] + 1][k] = SortII[NoCustomers + 1];
										Route[Counter[k]][k] = SortI[j];
										for (i = Counter[k]; i <= Counter[k] + 1; i++) {
											yPtr[i][k] = TempY[Route[i][k]][k];
											uPtr[i][k] = TempU[Route[i][k]];
											qPtr[i][k] = TempQ[Route[i][k]][k];
										}
										vPtr[Counter[k] + 1][Technology[SortII[NoCustomers + 1]]][k] = TempV[Route[Counter[k] + 1][k]][Technology[SortII[NoCustomers + 1]]][k];
										qPtr[Counter[k] + 2][k] = qPtr[Counter[k] + 1][k] - Demand[Route[Counter[k] + 1][k]];
										Counter[k] += 2;
										CounterStation[SortII[NoCustomers + 1]] ++;
										Seed[SortI[j]] = 1;
										RoutedCustomers++;
									}
								}
							}
							else {
								goto Label2;
							}
						}
					}
				}
			}
		}
	}
}

void RemoveRoute() {
	int i, j, k;
	float Cost2, Cost4, Cost1, Cost3;

	for (j = 1; j <= NoRoutes; j++) {
		if (Counter[j] == 0) {
			Cost1 = MaxEnergyCapacity[j];
			Cost2 = TCost[j];
			Cost3 = AcquisitionCost[j];
			Cost4 = EnergyConsumption[j];
			for (k = j + 1; k <= NoRoutes; k++) {
				for (i = 0; i <= Counter[k]; i++) {
					if (Route[i][k - 1] > NoCustomers&& Route[i][k - 1] < NoVertices) {
						vPtr[i][Technology[Route[i][k - 1]]][k - 1] = 0;
					}
					if (Route[i][k] > NoCustomers&& Route[i][k] < NoVertices) {
						vPtr[i][Technology[Route[i][k]]][k - 1] = vPtr[i][Technology[Route[i][k]]][k];
					}
					Route[i][k - 1] = Route[i][k];
					qPtr[i][k - 1] = qPtr[i][k];
					yPtr[i][k - 1] = yPtr[i][k];
					uPtr[i][k - 1] = uPtr[i][k];
				}
				if (Route[Counter[k] + 1][k - 1] != 0) {
					for (int l = Counter[k] + 1; l <= Counter[k - 1]; l++) {
						if (Route[l][k - 1] > NoCustomers&& Route[l][k - 1] < NoVertices) {
							vPtr[l][Technology[Route[l][k - 1]]][k - 1] = 0;
						}
						Route[l][k - 1] = 0;
						qPtr[l][k - 1] = 0;
						yPtr[l][k - 1] = 0;
						uPtr[l][k - 1] = 0;

					}
				}
				Counter[k - 1] = Counter[k];
				RouteLength[k - 1] = RouteLength[k];
				MaxEnergyCapacity[k - 1] = MaxEnergyCapacity[k];
				TCost[k - 1] = TCost[k];
				AcquisitionCost[k - 1] = AcquisitionCost[k];
				EnergyConsumption[k - 1] = EnergyConsumption[k];

			}
			for (i = 1; i <= Counter[NoRoutes]; i++) {
				if (Route[i][NoRoutes] > NoCustomers&& Route[i][NoRoutes] < NoVertices) {
					vPtr[i][Technology[Route[i][NoRoutes]]][NoRoutes] = 0;
				}
				Route[i][NoRoutes] = 0;
				qPtr[i][NoRoutes] = 0;
				yPtr[i][NoRoutes] = 0;
				uPtr[i][NoRoutes] = 0;

			}
			MaxEnergyCapacity[NoRoutes] = Cost1;
			TCost[NoRoutes] = Cost2;
			AcquisitionCost[NoRoutes] = Cost3;
			EnergyConsumption[NoRoutes] = Cost4;
			Counter[NoRoutes] = 0;
			RouteLength[NoRoutes] = 0;
			j = 0;
			NoRoutes--;
		}
	}
}

void RemoveCustomer(int Variable1, int Variable2) {

	int i, j, m;
	int Change = 0;
	float LengthDifference = 0;

	for (i = Variable2; i <= Counter[Variable1]; i++) {
		for (m = 1; m <= NoChargers; m++) {
			Cost -= vPtr[i][m][Variable1] * RCost[m];
		}
	}

	CounterRemoval++;
	RoutedCustomers--;
	RemovedCustomer[CounterRemoval] = Route[Variable2][Variable1];
	LengthDifference = d[Route[Variable2 - 1][Variable1]][Route[Variable2 + 1][Variable1]] - d[Route[Variable2 - 1][Variable1]][Route[Variable2][Variable1]] - d[Route[Variable2][Variable1]][Route[Variable2 + 1][Variable1]];
	RouteLength[Variable1] += LengthDifference;
	if (RouteLength[Variable1] < 0.1) {
		RouteLength[Variable1] = 0;
	}
	//Cost += LengthDifference; 
	if (MaxEnergyCapacity[Variable1] < EnergyLimit) {
		Cost += RCost[1] * yPtr[Counter[Variable1]][Variable1];
	}
	//else {
	//	Emissions +=  (Lambda * CurbWeight * Rho * Sigma2nd * (LengthDifference)); 
	//	Emissions += (Lambda * EngineFriction * EngineSpeed * EngineDisplacement * (LengthDifference/speed));
	//	Emissions += (Lambda * Epsilon2nd * Rho * LengthDifference * speed * speed);
	//	Emissions += (Rho * Sigma2nd * Lambda * ( qPtr[Variable2 - 1][Variable1] * d[Route[Variable2 - 1][Variable1]][Route[Variable2 + 1][Variable1]] - qPtr[Variable2 - 1][Variable1] * d[Route[Variable2 - 1][Variable1]][Route[Variable2][Variable1]] - qPtr[Variable2][Variable1] * d[Route[Variable2][Variable1]][Route[Variable2 + 1][Variable1]]));
	//	for (i = Variable2; i <= Counter[Variable1]-1; i++) {
	//		Emissions -= Rho * Sigma2nd * Lambda * d[Route[i][Variable1]][Route[i+1][Variable1]] * Demand[RemovedCustomer[CounterRemoval]];
	//	}
	//}
	Cost += (TCost[Variable1] * (LengthDifference));
	if (RouteLength[Variable1] != 0) {
		for (int i = Variable2; i <= Counter[Variable1]; i++) {
			qPtr[i][Variable1] = qPtr[i + 1][Variable1] + Demand[RemovedCustomer[CounterRemoval]];
			if (Route[i - 1][Variable1] <= NoCustomers || Route[i - 1][Variable1] > NoVertices) {
				if (Change == 0) {
					yPtr[i][Variable1] = yPtr[i + 1][Variable1] - EnergyConsumption[Variable1] * LengthDifference;
					if (i == Variable2) {
						if (eTime[Route[i - 1][Variable1]] <= uPtr[i - 1][Variable1]) {
							uPtr[i][Variable1] = uPtr[i - 1][Variable1] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
						}
						else {
							uPtr[i][Variable1] = eTime[Route[i - 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
						}
					}
					else {
						if (eTime[Route[i - 1][Variable1]] <= uPtr[i - 1][Variable1]) {
							uPtr[i][Variable1] = uPtr[i - 1][Variable1] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
						}
						else {
							uPtr[i][Variable1] = eTime[Route[i - 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
						}
					}
				}
				else {
					yPtr[i][Variable1] = yPtr[i - 1][Variable1] - EnergyConsumption[Variable1] * d[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
					if (eTime[Route[i - 1][Variable1]] <= uPtr[i - 1][Variable1]) {
						uPtr[i][Variable1] = uPtr[i - 1][Variable1] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
					}
					else {
						uPtr[i][Variable1] = eTime[Route[i - 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
					}
				}
			}
			else {
				Change = 1;
				if (Route[i - 1][Variable1] == Route[i + 1][Variable1]) {
					CounterStation[Route[i - 1][Variable1]] --;
					for (j = i + 1; j <= Counter[Variable1]; j++) {
						Route[j - 1][Variable1] = Route[j][Variable1];
						qPtr[j - 1][Variable1] = qPtr[j][Variable1];
						uPtr[j - 1][Variable1] = uPtr[j][Variable1];
						yPtr[j - 1][Variable1] = yPtr[j][Variable1];
						vPtr[j - 1][Technology[Route[i - 1][Variable1]]][Variable1] = vPtr[j][Technology[Route[i - 1][Variable1]]][Variable1];
					}
					Route[Counter[Variable1]][Variable1] = 0;
					qPtr[Counter[Variable1]][Variable1] = 0;
					uPtr[Counter[Variable1]][Variable1] = 0;
					yPtr[Counter[Variable1]][Variable1] = 0;
					vPtr[Counter[Variable1]][Technology[Route[i - 1][Variable1]]][Variable1] = 0;
					Counter[Variable1]--;
				}
				if (Technology[Route[i - 1][Variable1]] > 0 && Technology[Route[i - 1][Variable1]] <= 3) {    //  Update Energy for the station
					if (yPtr[i - 1][Variable1] < EnergyLimit) {
						if (yPtr[i - 1][Variable1] > 0) {
							vPtr[i - 1][Technology[Route[i - 1][Variable1]]][Variable1] = 0.8 * MaxEnergyCapacity[Variable1] - yPtr[i - 1][Variable1];
							yPtr[i][Variable1] = yPtr[i - 1][Variable1] + vPtr[i - 1][Technology[Route[i - 1][Variable1]]][Variable1] - EnergyConsumption[Variable1] * d[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
						}
						else {
							vPtr[i - 1][Technology[Route[i - 1][Variable1]]][Variable1] = 0.8 * MaxEnergyCapacity[Variable1];
							yPtr[i][Variable1] = vPtr[i - 1][Technology[Route[i - 1][Variable1]]][Variable1] - EnergyConsumption[Variable1] * d[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
						}
					}
					vPtr[i][Technology[Route[i - 1][Variable1]]][Variable1] = 0;
				}
				if (eTime[Route[i - 1][Variable1]] <= uPtr[i - 1][Variable1]) {
					if (Technology[Route[i - 1][Variable1]] > 0 && Technology[Route[i - 1][Variable1]] <= 3) {    //  Update Time for the station
						uPtr[i][Variable1] = uPtr[i - 1][Variable1] + vPtr[i - 1][Technology[Route[i - 1][Variable1]]][Variable1] * Rate[Route[i - 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
					}
				}
				else {
					if (Technology[Route[Counter[Variable1] - 1][Variable1]] > 0 && Technology[Route[Counter[Variable1] - 1][Variable1]] <= 3) {    //  Update Time for the station
						uPtr[i][Variable1] = eTime[Route[i - 1][Variable1]] + vPtr[i - 1][Technology[Route[i - 1][Variable1]]][Variable1] * Rate[Route[i - 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
					}
				}
			}
			Route[i][Variable1] = Route[i + 1][Variable1];
			/// Take care of the last node
			/// Check if it makes route infeasible in terms of energy, load or time
		}
		Route[Counter[Variable1]][Variable1] = 0;
		yPtr[Counter[Variable1]][Variable1] = 0;
		qPtr[Counter[Variable1]][Variable1] = 0;
		uPtr[Counter[Variable1]][Variable1] = 0;
		Counter[Variable1]--;
		if (MaxEnergyCapacity[Variable1] < EnergyLimit) {
			Cost -= (RCost[1] * yPtr[Counter[Variable1]][Variable1]);
		}
		for (j = Variable2; j <= Counter[Variable1]; j++) {
			for (m = 1; m <= NoChargers; m++) {
				Cost += vPtr[j][m][Variable1] * RCost[m];
			}
		}
	}
	else {
		if (MaxEnergyCapacity[Variable1] < EnergyLimit) {
			Cost -= RCost[1] * Ratio * MaxEnergyCapacity[Variable1];
			Cost -= AcquisitionCost[Variable1];
			NoEV--;
		}
		else {
			Cost -= AcquisitionCost[Variable1];
			NoCV--;
		}

		for (int i = Variable2; i <= Counter[Variable1]; i++) {
			Route[i][Variable1] = 0;
			qPtr[i][Variable1] = 0;
			yPtr[i][Variable1] = 0;
			uPtr[i][Variable1] = 0;
		}
		Counter[Variable1] = 0;
		RemoveRoute();

	}

}

void RandomCustomerRemoval1() {
	int i, j, k, SelectedCustomer, SelectedRoute, * RemovalCustomer;

	RemovalCustomer = new int[NoCustomers + 1];
	for (i = 0; i <= NoCustomers; i++) {
		RemovalCustomer[i] = 0;
	}

	for (i = 1; i <= NoRemovalC; i++) {
		RemovalCustomer[i] = rand() % NoCustomers + 1;
		for (j = 1; j < i; j++) {
			if (RemovalCustomer[i] == RemovalCustomer[j]) {
				i--;
			}
		}
	}
	// Remove the customers from the routes
	for (j = 1; j <= NoRemovalC; j++) {
		for (k = 1; k <= NoVehicles; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (RemovalCustomer[j] == Route[i][k]) {
					SelectedCustomer = i;
					SelectedRoute = k;
					RemoveCustomer(SelectedRoute, SelectedCustomer);

				}
			}
		}
	}
}

void WorstDistanceRemoval2() {
	int i, j, k, l, SelectedCustomer, SelectedRoute;
	float* DistanceCost;

	DistanceCost = new float[NoCustomers + 1];
	for (i = 0; i <= NoCustomers; i++) {
		DistanceCost[i] = 0;
	}

	for (k = 1; k <= NoRoutes; k++) {
		for (i = 1; i <= Counter[k]; i++) {
			if (Route[i][k] <= NoCustomers) {
				DistanceCost[Route[i][k]] = d[Route[i - 1][k]][Route[i][k]] + d[Route[i][k]][Route[i + 1][k]];
			}
		}
	}

	for (i = 1; i <= NoCustomers - 1; i++) {
		for (j = i + 1; j <= NoCustomers; j++) {
			if (DistanceCost[SortI[i]] > DistanceCost[SortI[j]]) {
				//cout<<lTime[i]<<endl<<lTime[j]<<endl;
				int temp;
				temp = SortI[j];
				for (l = j - 1; l >= i; l--) {
					SortI[l + 1] = SortI[l];
					//cout<<Sort[k+1]<<endl;
				}
				SortI[i] = temp;
				//cout<<SortI[j]<<endl;
			}
		}
	}
	for (j = NoCustomers; j > NoCustomers - NoRemovalC; j--) {
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (SortI[j] == Route[i][k]) {
					SelectedCustomer = i;
					SelectedRoute = k;
					RemoveCustomer(SelectedRoute, SelectedCustomer);
				}
			}
		}
	}

}

void WorstTimeRemoval3() {
	int i, j, k, l, SelectedCustomer, SelectedRoute;
	float* TimeWindowCost;

	TimeWindowCost = new float[NoCustomers + 1];

	for (i = 0; i <= NoCustomers; i++) {
		TimeWindowCost[i] = 0;
	}

	for (k = 1; k <= NoRoutes; k++) {
		for (i = 1; i <= Counter[k]; i++) {
			if (Route[i][k] <= NoCustomers) {
				TimeWindowCost[Route[i][k]] = lTime[Route[i][k]] - eTime[Route[i][k]];
			}
		}
	}

	for (i = 1; i <= NoCustomers - 1; i++) {
		for (j = i + 1; j <= NoCustomers; j++) {
			if (TimeWindowCost[SortI[i]] > TimeWindowCost[SortI[j]]) {
				//cout<<lTime[i]<<endl<<lTime[j]<<endl;
				int temp;
				temp = SortI[j];
				for (l = j - 1; l >= i; l--) {
					SortI[l + 1] = SortI[l];
					//cout<<Sort[k+1]<<endl;
				}
				SortI[i] = temp;
				//cout<<SortI[j]<<endl;
			}
		}
	}
	for (j = NoCustomers; j > NoCustomers - NoRemovalC; j--) {
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] == SortI[j]) {
					SelectedCustomer = i;
					SelectedRoute = k;
					RemoveCustomer(SelectedRoute, SelectedCustomer);
				}
			}
		}
	}
}

void ShawRemoval4() {
	int i, k, j, l, m, SelectedCustomer, SelectedRoute, RouteNumber;
	float Fi1, Fi2, Fi3, Fi4, ** Relatedness;
	Fi1 = 0.5;
	Fi2 = 13;  ///????
	Fi3 = 0.15;
	Fi4 = 0.25;
	RouteNumber = 0;


	Relatedness = new float* [NoCustomers + 1];
	for (i = 0; i <= NoCustomers; i++) {
		Relatedness[i] = new float[NoCustomers + 1];
	}
	for (i = 0; i <= NoCustomers; i++) {
		for (j = 0; j <= NoCustomers; j++) {
			Relatedness[i][j] = 0;
		}
	}
	int RandCustomer = rand() % NoCustomers + 1;
	for (j = 1; j <= NoCustomers; j++) {
		if (SortI[j] == RandCustomer) {
			for (i = j - 1; i >= 1; i--) {
				SortI[i + 1] = SortI[i];
			}
			SortI[1] = RandCustomer;
		}
	}
	for (m = 1; m <= NoRemovalC; m++) {
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] == SortI[m]) {
					RouteNumber = k;
				}
			}
		}
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] != SortI[m] && Route[i][k] <= NoCustomers) {
					Relatedness[Route[i][k]][SortI[m]] = Fi1 * d[Route[i][k]][SortI[m]] + Fi2 * abs(eTime[Route[i][k]] - eTime[SortI[m]]) + Fi4 * abs(Demand[Route[i][k]] - Demand[SortI[m]]);
					if (k == RouteNumber) {
						Relatedness[Route[i][k]][SortI[m]] -= Fi3;
					}
					else {
						Relatedness[Route[i][k]][SortI[m]] += Fi3;
					}
				}
			}
		}
		for (i = m; i <= NoCustomers - 1; i++) {
			for (j = i + 1; j <= NoCustomers; j++) {
				if (Relatedness[SortI[i]][SortI[m]] > Relatedness[SortI[j]][SortI[m]]) {
					//cout<<lTime[i]<<endl<<lTime[j]<<endl;
					int temp;
					temp = SortI[j];
					for (l = j - 1; l >= i; l--) {
						SortI[l + 1] = SortI[l];
						//cout<<Sort[k+1]<<endl;
					}
					SortI[i] = temp;
					//cout<<SortI[j]<<endl;
				}
			}
		}
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] == SortI[m]) {
					SelectedCustomer = i;
					SelectedRoute = k;
					RemoveCustomer(SelectedRoute, SelectedCustomer);
				}
			}
		}
	}
}

void ProximityRemoval5() {
	int i, k, j, l, m, SelectedCustomer, SelectedRoute, RouteNumber;
	float Fi1, ** Relatedness;
	Fi1 = 1;
	RouteNumber = 0;

	Relatedness = new float* [NoCustomers + 1];
	for (i = 0; i <= NoCustomers; i++) {
		Relatedness[i] = new float[NoCustomers + 1];
	}
	for (i = 0; i <= NoCustomers; i++) {
		for (j = 0; j <= NoCustomers; j++) {
			Relatedness[i][j] = 0;
		}
	}
	int RandCustomer = rand() % NoCustomers + 1;
	for (j = 1; j <= NoCustomers; j++) {
		if (SortI[j] == RandCustomer) {
			for (i = j - 1; i >= 1; i--) {
				SortI[i + 1] = SortI[i];
			}
			SortI[1] = RandCustomer;
		}
	}
	for (m = 1; m <= NoRemovalC; m++) {
		for (k = 1; k <= NoVehicles; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] == SortI[m]) {
					RouteNumber = k;
				}
			}
		}
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] != SortI[m] && Route[i][k] <= NoCustomers) {
					Relatedness[Route[i][k]][SortI[m]] = Fi1 * d[Route[i][k]][SortI[m]];
				}
			}
		}
		for (i = m; i <= NoCustomers - 1; i++) {
			for (j = i + 1; j <= NoCustomers; j++) {
				if (Relatedness[SortI[i]][SortI[m]] > Relatedness[SortI[j]][SortI[m]]) {
					//cout<<lTime[i]<<endl<<lTime[j]<<endl;
					int temp;
					temp = SortI[j];
					for (l = j - 1; l >= i; l--) {
						SortI[l + 1] = SortI[l];
						//cout<<Sort[k+1]<<endl;
					}
					SortI[i] = temp;
					//cout<<SortI[j]<<endl;
				}
			}
		}
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] == SortI[m]) {
					SelectedCustomer = i;
					SelectedRoute = k;
					RemoveCustomer(SelectedRoute, SelectedCustomer);
				}
			}
		}
	}
}

void DemandRemoval6() {
	int i, k, j, l, m, SelectedCustomer, SelectedRoute, RouteNumber;
	float Fi4, ** Relatedness;
	Fi4 = 1;
	RouteNumber = 0;

	Relatedness = new float* [NoCustomers + 1];
	for (i = 0; i <= NoCustomers; i++) {
		Relatedness[i] = new float[NoCustomers + 1];
	}
	for (i = 0; i <= NoCustomers; i++) {
		for (j = 0; j <= NoCustomers; j++) {
			Relatedness[i][j] = 0;
		}
	}
	int RandCustomer = rand() % NoCustomers + 1;
	for (j = 1; j <= NoCustomers; j++) {
		if (SortI[j] == RandCustomer) {
			for (i = j - 1; i >= 1; i--) {
				SortI[i + 1] = SortI[i];
			}
			SortI[1] = RandCustomer;
		}
	}
	for (m = 1; m <= NoRemovalC; m++) {
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] == SortI[m]) {
					RouteNumber = k;
				}
			}
		}
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] != SortI[m] && Route[i][k] <= NoCustomers) {
					Relatedness[Route[i][k]][SortI[m]] = Fi4 * abs(Demand[Route[i][k]] - Demand[SortI[m]]);
				}
			}
		}
		for (i = m; i <= NoCustomers - 1; i++) {
			for (j = i + 1; j <= NoCustomers; j++) {
				if (Relatedness[SortI[i]][SortI[m]] > Relatedness[SortI[j]][SortI[m]]) {
					//cout<<lTime[i]<<endl<<lTime[j]<<endl;
					int temp;
					temp = SortI[j];
					for (l = j - 1; l >= i; l--) {
						SortI[l + 1] = SortI[l];
						//cout<<Sort[k+1]<<endl;
					}
					SortI[i] = temp;
					//cout<<SortI[j]<<endl;
				}
			}
		}
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] == SortI[m]) {
					SelectedCustomer = i;
					SelectedRoute = k;
					RemoveCustomer(SelectedRoute, SelectedCustomer);
				}
			}
		}
	}
}

void TimeBasedRemoval7() {
	int i, k, j, l, m, SelectedCustomer, SelectedRoute, RouteNumber;
	float Fi2, ** Relatedness;
	Fi2 = 1;
	RouteNumber = 0;

	Relatedness = new float* [NoCustomers + 1];
	for (i = 0; i <= NoCustomers; i++) {
		Relatedness[i] = new float[NoCustomers + 1];
	}
	for (i = 0; i <= NoCustomers; i++) {
		for (j = 0; j <= NoCustomers; j++) {
			Relatedness[i][j] = 0;
		}
	}
	int RandCustomer = rand() % NoCustomers + 1;
	for (j = 1; j <= NoCustomers; j++) {
		if (SortI[j] == RandCustomer) {
			for (i = j - 1; i >= 1; i--) {
				SortI[i + 1] = SortI[i];
			}
			SortI[1] = RandCustomer;
		}
	}
	for (m = 1; m <= NoRemovalC; m++) {
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] == SortI[m]) {
					RouteNumber = k;
				}
			}
		}
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] != SortI[m] && Route[i][k] <= NoCustomers) {
					Relatedness[Route[i][k]][SortI[m]] = Fi2 * abs(eTime[Route[i][k]] - eTime[SortI[m]]);
				}
			}
		}
		for (i = m; i <= NoCustomers - 1; i++) {
			for (j = i + 1; j <= NoCustomers; j++) {
				if (Relatedness[SortI[i]][SortI[m]] > Relatedness[SortI[j]][SortI[m]]) {
					//cout<<lTime[i]<<endl<<lTime[j]<<endl;
					int temp;
					temp = SortI[j];
					for (l = j - 1; l >= i; l--) {
						SortI[l + 1] = SortI[l];
						//cout<<Sort[k+1]<<endl;
					}
					SortI[i] = temp;
					//cout<<SortI[j]<<endl;
				}
			}
		}
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] == SortI[m]) {
					SelectedCustomer = i;
					SelectedRoute = k;
					RemoveCustomer(SelectedRoute, SelectedCustomer);
				}
			}
		}
	}
}

void RemoveStation(int Variable1, int Variable2) {

	int Change = 0;
	float LengthDifference = 0;
	// Update cost by removing the costs that the station added to the total cost
	LengthDifference = d[Route[Variable2 - 1][Variable1]][Route[Variable2 + 1][Variable1]] - d[Route[Variable2 - 1][Variable1]][Route[Variable2][Variable1]] - d[Route[Variable2][Variable1]][Route[Variable2 + 1][Variable1]];
	RouteLength[Variable1] += LengthDifference;
	if (RouteLength[Variable1] < 0.01) {
		RouteLength[Variable1] = 0;
	}

	//Cost = Cost + d[Route[Variable2-1][Variable1]][Route[Variable2+1][Variable1]] - d[Route[Variable2-1][Variable1]][Route[Variable2][Variable1]] - d[Route[Variable2][Variable1]][Route[Variable2+1][Variable1]];
	if (MaxEnergyCapacity[Variable1] < EnergyLimit) {
		Cost += RCost[1] * yPtr[Counter[Variable1]][Variable1];
	}
	Cost += (TCost[Variable1] * (LengthDifference));
	for (int i = Variable2; i <= Counter[Variable1] - 1; i++) {
		if (Route[i][Variable1] > NoCustomers) {
			if (Technology[Route[i][Variable1]] > 0 && Technology[Route[i][Variable1]] <= 3) {
				Cost -= RCost[Technology[Route[i][Variable1]]] * vPtr[i][Technology[Route[i][Variable1]]][Variable1];
				vPtr[i][Technology[Route[i][Variable1]]][Variable1] = 0;
			}
		}
	}
	CounterStation[Route[Variable2][Variable1]]--;
	if (RouteLength[Variable1] > 0) {
		// Update Load, Energy and Time until the next station
		for (int i = Variable2; i <= Counter[Variable1]; i++) {
			qPtr[i][Variable1] = qPtr[i + 1][Variable1];
			if (Route[i - 1][Variable1] <= NoCustomers && Route[i - 1][Variable1] > 0) {
				if (Change == 0) {
					yPtr[i][Variable1] = yPtr[i - 1][Variable1] - EnergyConsumption[Variable1] * d[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
					if (eTime[Route[i - 1][Variable1]] <= uPtr[i - 1][Variable1]) {
						uPtr[i][Variable1] = uPtr[i - 1][Variable1] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]];
					}
					else {
						uPtr[i][Variable1] = eTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]];
					}
				}
				else {
					yPtr[i][Variable1] = yPtr[i - 1][Variable1] - EnergyConsumption[Variable1] * d[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
					if (eTime[Route[i - 1][Variable1]] <= uPtr[i - 1][Variable1]) {
						uPtr[i][Variable1] = uPtr[i - 1][Variable1] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
					}
					else {
						uPtr[i][Variable1] = eTime[Route[i - 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
					}
				}
			}
			else {
				// If it reaches to a station, energy and time will update differently afterwards
				Change = 1;
				if (yPtr[i - 1][Variable1] < EnergyLimit) {
					if (yPtr[i - 1][Variable1] < 0) {
						// Update next station energy level by considering full amount of charge if previous energy level is negative
						if (Technology[Route[i - 1][Variable1]] > 0 && Technology[Route[i - 1][Variable1]] <= 3) {    //  Update Energy for the station
							vPtr[i - 1][Technology[Route[i - 1][Variable1]]][Variable1] = 0.8 * MaxEnergyCapacity[Variable1];
							yPtr[i][Variable1] = vPtr[i - 1][Technology[Route[i - 1][Variable1]]][Variable1] - EnergyConsumption[Variable1] * d[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
						}
					}
					else {
						//
						if (Technology[Route[i - 1][Variable1]] > 0 && Technology[Route[i - 1][Variable1]] <= 3) {
							vPtr[i - 1][Technology[Route[i - 1][Variable1]]][Variable1] = 0.8 * MaxEnergyCapacity[Variable1] - yPtr[i - 1][Variable1];
							yPtr[i][Variable1] = yPtr[i - 1][Variable1] + vPtr[i - 1][Technology[Route[i - 1][Variable1]]][Variable1] - EnergyConsumption[Variable1] * d[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
						}
						else {
							// If previous route is the depot, it should be upadate like customers not like stations
							if (Route[i - 1][Variable1] == 0) {
								yPtr[i][Variable1] = yPtr[i - 1][Variable1] - EnergyConsumption[Variable1] * d[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
							}
						}
					}
				}
				if (eTime[Route[i - 1][Variable1]] <= uPtr[i - 1][Variable1]) {
					if (Technology[Route[i - 1][Variable1]] > 0 && Technology[Route[i - 1][Variable1]] <= 3) {    //  Update Time for the station
						uPtr[i][Variable1] = uPtr[i - 1][Variable1] + vPtr[i - 1][Technology[Route[i - 1][Variable1]]][Variable1] * Rate[Route[i - 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
					}
					else {
						if (Route[i - 1][Variable1] == 0) {
							uPtr[i][Variable1] = uPtr[i - 1][Variable1] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
						}
					}
				}
				else {
					if (Technology[Route[Counter[Variable1] - 1][Variable1]] > 0 && Technology[Route[Counter[Variable1] - 1][Variable1]] <= 3) {    //  Update Time for the station
						uPtr[i][Variable1] = eTime[Route[i - 1][Variable1]] + vPtr[i - 1][Technology[Route[i - 1][Variable1]]][Variable1] * Rate[Route[i - 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i + 1][Variable1]];
					}
				}
			}
			Route[i][Variable1] = Route[i + 1][Variable1];
		}
		// Update the route
		Route[Counter[Variable1]][Variable1] = 0;
		yPtr[Counter[Variable1]][Variable1] = 0;
		qPtr[Counter[Variable1]][Variable1] = 0;
		uPtr[Counter[Variable1]][Variable1] = 0;
		Counter[Variable1]--;
		if (MaxEnergyCapacity[Variable1] < EnergyLimit) {
			Cost -= RCost[1] * yPtr[Counter[Variable1]][Variable1];
		}
		for (int i = Variable2; i <= Counter[Variable1] - 1; i++) {
			if (Route[i][Variable1] > NoCustomers) {
				if (Technology[Route[i][Variable1]] > 0 && Technology[Route[i][Variable1]] <= 3) {
					Cost += RCost[Technology[Route[i][Variable1]]] * vPtr[i][Technology[Route[i][Variable1]]][Variable1];
				}
			}
		}
	}
	// If lenght of the route is zero it means that by removing the station, the route which contatined just the removed station should be removed completely
	else {
		if (MaxEnergyCapacity[Variable1] < EnergyLimit) {
			Cost -= RCost[1] * Ratio * MaxEnergyCapacity[Variable1];
			Cost -= AcquisitionCost[Variable1];
			NoEV--;
		}
		else {
			Cost -= AcquisitionCost[Variable1];
			NoCV--;
		}
		for (int i = Variable2; i <= Counter[Variable1]; i++) {
			Route[i][Variable1] = 0;
			qPtr[i][Variable1] = 0;
			yPtr[i][Variable1] = 0;
			uPtr[i][Variable1] = 0;
		}
		Counter[Variable1] = 0;
		// Update routes
		RemoveRoute();
	}
}

void RCwPS() {
	int i, j, k, SelectedRoute, SelectedStation;

	for (j = 1; j <= CounterRemoval; j++) {
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] == RemovedCustomer[j]) {
					if (Route[i - 1][k] > NoCustomers&& Route[i - 1][k] < NoVertices) {
						SelectedRoute = k;
						SelectedStation = i - 1;
						RemoveStation(SelectedRoute, SelectedStation);
					}
				}
			}
		}
	}
}

void RCwSS() {
	int i, j, k, SelectedRoute, SelectedStation;

	for (j = 1; j <= CounterRemoval; j++) {
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				if (Route[i][k] == RemovedCustomer[j]) {
					if (Route[i + 1][k] > NoCustomers&& Route[i + 1][k] < NoVertices) {
						SelectedRoute = k;
						SelectedStation = i + 1;
						RemoveStation(SelectedRoute, SelectedStation);
					}
				}
			}
		}
	}
}

void StationVicinityRemoval8() {
	int i, k, SelectedStation, SelectedCustomer, SelectedRoute, Max, Min;
	float r, MaxPercent, MinPercent;
	MinPercent = 0.2;
	MaxPercent = 0.8;

	Max = MaxPercent * MaxDistance - MinPercent * MaxDistance;
	Min = MinPercent * MaxDistance;
	r = rand() % Max + Min + 1;
	SelectedStation = rand() % NoStations + NoCustomers + 1;
	for (k = 1; k <= NoRoutes; k++) {
		for (i = 1; i <= Counter[k]; i++) {
			if (Route[i][k] <= NoCustomers && Route[i][k] > 0) {
				if (d[Route[i][k]][SelectedStation] < r) {
					SelectedCustomer = i;
					SelectedRoute = k;
					RemoveCustomer(SelectedRoute, SelectedCustomer);
				}
			}
		}
	}
}

void RandomRouteRemoval1() {
	int i, j, k, SelectedRoute;
	float PercentRemovalR;
	PercentRemovalR = 0.4;
	// Select the routes to be removed
	int NoRemovalR = floor(PercentRemovalR * NoRoutes);
	for (i = 1; i <= NoRemovalR; i++) {
		RemovalRoute[i] = rand() % NoRoutes + 1;
		for (j = 1; j < i; j++) {
			if (RemovalRoute[i] == RemovalRoute[j]) {
				i--;
			}
		}
	}
	// Remove the routes
	for (j = 1; j <= NoRemovalR; j++) {
		if (MaxEnergyCapacity[RemovalRoute[j]] < EnergyLimit) {
			Cost -= RCost[1] * Ratio * MaxEnergyCapacity[RemovalRoute[j]];
			Cost += RCost[1] * yPtr[Counter[RemovalRoute[j]]][RemovalRoute[j]];
			Cost -= AcquisitionCost[RemovalRoute[j]];
			NoEV--;
		}
		else {
			Cost -= AcquisitionCost[RemovalRoute[j]];
			NoCV--;
		}

		for (i = 1; i <= Counter[RemovalRoute[j]]; i++) {
			Cost -= TCost[RemovalRoute[j]] * d[Route[i - 1][RemovalRoute[j]]][Route[i][RemovalRoute[j]]];
			//if (MaxEnergyCapacity[RemovalRoute[j]] > EnergyLimit) {
			//	Emissions -= (Lambda * CurbWeight * Rho * Sigma2nd * (d[Route[i - 1][RemovalRoute[j]]][Route[i][RemovalRoute[j]]]));
			//	Emissions -= (Lambda * EngineFriction * EngineSpeed * EngineDisplacement * (d[Route[i - 1][RemovalRoute[j]]][Route[i][RemovalRoute[j]]]/speed));
			//	Emissions -= (Lambda * Epsilon2nd * Rho * d[Route[i - 1][RemovalRoute[j]]][Route[i][RemovalRoute[j]]] * speed * speed);
			//	Emissions -= (Rho * Sigma2nd * Lambda * ( qPtr[i - 1][RemovalRoute[j]] * d[Route[i - 1][RemovalRoute[j]]][Route[i][RemovalRoute[j]]]));
			//}
			//Cost = Cost - d[Route[i-1][RemovalRoute[j]]][Route[i][RemovalRoute[j]]];
			if (Route[i][RemovalRoute[j]] <= NoCustomers) {
				RoutedCustomers--;
				CounterRemoval++;
				RemovedCustomer[CounterRemoval] = Route[i][RemovalRoute[j]];
			}
			if (Route[i][RemovalRoute[j]] > NoCustomers&& Route[i][RemovalRoute[j]] < NoVertices) {
				CounterStation[Route[i][RemovalRoute[j]]] --;
			}
			if (MaxEnergyCapacity[RemovalRoute[j]] < EnergyLimit) {
				if (Route[i][RemovalRoute[j]] > NoCustomers&& Route[i][RemovalRoute[j]] < NoVertices) {
					if (Technology[Route[i][RemovalRoute[j]]] > 0 && Technology[Route[i][RemovalRoute[j]]] <= 3) {
						Cost -= RCost[Technology[Route[i][RemovalRoute[j]]]] * vPtr[i][Technology[Route[i][RemovalRoute[j]]]][RemovalRoute[j]];

					}
				}
			}
		}
		for (i = 1; i <= Counter[RemovalRoute[j]]; i++) {
			if (Route[i][RemovalRoute[j]] > NoCustomers&& Route[i][RemovalRoute[j]] < NoVertices) {
				vPtr[i][Technology[Route[i][RemovalRoute[j]]]][RemovalRoute[j]] = 0;
			}
			Route[i][RemovalRoute[j]] = 0;
			qPtr[i][RemovalRoute[j]] = 0;
			yPtr[i][RemovalRoute[j]] = 0;
			uPtr[i][RemovalRoute[j]] = 0;
		}
		Counter[RemovalRoute[j]] = 0;
		RouteLength[RemovalRoute[j]] = 0;
	}
	RemoveRoute();
}

void GreedyRouteRemoval2() {
	int i, j, k, SelectedRoute, Min;
	float PercentRemovalR;
	PercentRemovalR = 0.6;
	Min = BigNumber;

	int NoRemovalR = floor(PercentRemovalR * NoRoutes);
	// Remove the routes
	for (j = 1; j <= NoRemovalR; j++) {
		Min = BigNumber;
		for (k = 1; k <= NoRoutes; k++) {
			if (Counter[k] <= Min && Counter[k] != 0) {
				SelectedRoute = k;
				Min = Counter[k];
			}
		}
		if (MaxEnergyCapacity[SelectedRoute] < EnergyLimit) {
			Cost -= RCost[1] * Ratio * MaxEnergyCapacity[SelectedRoute];
			Cost += RCost[1] * yPtr[Counter[SelectedRoute]][SelectedRoute];
			Cost -= AcquisitionCost[SelectedRoute];
			NoEV--;
		}
		else {
			Cost -= AcquisitionCost[SelectedRoute];
			NoCV--;
		}
		for (i = 1; i <= Counter[SelectedRoute]; i++) {
			Cost -= (TCost[SelectedRoute] * d[Route[i - 1][SelectedRoute]][Route[i][SelectedRoute]]);
				//if (MaxEnergyCapacity[SelectedRoute] > EnergyLimit) {
				//	Emissions -= (Lambda * CurbWeight * Rho * Sigma2nd * (d[Route[i - 1][SelectedRoute]][Route[i][SelectedRoute]]));
				//	Emissions -= (Lambda * EngineFriction * EngineSpeed * EngineDisplacement * (d[Route[i - 1][SelectedRoute]][Route[i][SelectedRoute]]/speed));
				//	Emissions -= (Lambda * Epsilon2nd * Rho * d[Route[i - 1][SelectedRoute]][Route[i][SelectedRoute]] * speed * speed);
				//	Emissions -= (Rho * Sigma2nd * Lambda * (d[Route[i - 1][SelectedRoute]][Route[i][SelectedRoute]]) * qPtr[i-1][SelectedRoute]);
				//}
			//Cost -= d[Route[i-1][SelectedRoute]][Route[i][SelectedRoute]];
			if (Route[i][SelectedRoute] <= NoCustomers) {
				RoutedCustomers--;
				CounterRemoval++;
				RemovedCustomer[CounterRemoval] = Route[i][SelectedRoute];
			}
			if (Route[i][SelectedRoute] > NoCustomers&& Route[i][SelectedRoute] < NoVertices) {
				CounterStation[Route[i][SelectedRoute]] --;
			}
			if (MaxEnergyCapacity[SelectedRoute] < EnergyLimit) {
				if (Route[i][SelectedRoute] > NoCustomers&& Route[i][SelectedRoute] < NoVertices) {
					if (Technology[Route[i][SelectedRoute]] > 0 && Technology[Route[i][SelectedRoute]] <= 3) {
						Cost -= RCost[Technology[Route[i][SelectedRoute]]] * vPtr[i][Technology[Route[i][SelectedRoute]]][SelectedRoute];
					}
				}
			}
		}
		for (i = 1; i <= Counter[SelectedRoute]; i++) {
			if (Route[i][SelectedRoute] > NoCustomers&& Route[i][SelectedRoute] < NoVertices) {
				vPtr[i][Technology[Route[i][SelectedRoute]]][SelectedRoute] = 0;
			}
			Route[i][SelectedRoute] = 0;
			qPtr[i][SelectedRoute] = 0;
			yPtr[i][SelectedRoute] = 0;
			uPtr[i][SelectedRoute] = 0;
		}
		Counter[SelectedRoute] = 0;
		RouteLength[SelectedRoute] = 0;
	}
	RemoveRoute();
}

void RandomStationRemoval1() {
	int i, j, k, SelectedStation, SelectedRoute, NoUsedStations, NoRemovalS;
	NoUsedStations = 0;
	float PercentRemovalS = 0.45;
	for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
		NoUsedStations += CounterStation[i];
	}
	if (NoUsedStations > 0) {
	// Determine number of stations to be removed
	if (NoUsedStations < NoStations) {
		NoRemovalS = floor(PercentRemovalS * NoUsedStations);
	}
	else {
		NoRemovalS = floor(PercentRemovalS * NoStations);
	}
		// Select stations to be removed
		for (i = 1; i <= NoRemovalS; i++) {
			RemovalStation[i] = rand() % NoStations + NoCustomers + 1;
			for (j = 1; j < i; j++) {
				if (RemovalStation[i] == RemovalStation[j]) {
					i--;
				}
			}
		}
		// Remove the stations from the routes
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k]; i++) {
				for (j = 1; j <= NoRemovalS; j++) {
					if (Route[i][k] == RemovalStation[j]) {
						SelectedStation = i;
						SelectedRoute = k;
						RemoveStation(SelectedRoute, SelectedStation);

					}
				}
			}
		}
	}
}

void WorstDistanceStationRemoval2() {
	int i, j, k, l, SelectedStation, SelectedRoute, NoUsedStations, NoRemovalS;
	float PercentRemovalS, * DistanceCost;
	NoUsedStations = 0;
	PercentRemovalS = 0.4;
	for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
		NoUsedStations += CounterStation[i];
	}
	if (NoUsedStations > 0) {
		// Determine number of stations to be removed
		if (NoUsedStations < NoStations) {
			NoRemovalS = floor(PercentRemovalS * NoUsedStations);
		}
		else {
			NoRemovalS = floor(PercentRemovalS * NoStations);
		}

		// Calculate total distance from all of the customers for each station
		DistanceCost = new float[NoCustomers + NoStations + 1];
		for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
			DistanceCost[i] = 0;
		}
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 1; i <= Counter[k] - 1; i++) {
				if (Route[i][k] > NoCustomers) {
					DistanceCost[Route[i][k]] = DistanceCost[Route[i][k]] + d[Route[i - 1][k]][Route[i][k]] + d[Route[i][k]][Route[i + 1][k]];
				}
			}
		}

		// Sort stations based on their total distance
		for (i = NoCustomers; i <= NoCustomers + NoStations - 1; i++) {
			for (j = i + 1; j <= NoCustomers + NoStations; j++) {
				if (DistanceCost[SortII[i]] > DistanceCost[SortII[j]]) {
					int temp;
					temp = SortII[j];
					for (l = j - 1; l >= i; l--) {
						SortII[l + 1] = SortII[l];
					}
					SortII[i] = temp;
				}
			}
		}
		// Remove the stations from the routes
		for (j = NoCustomers + NoStations; j > NoCustomers + NoStations - NoRemovalS; j--) {
			for (k = 1; k <= NoRoutes; k++) {
				for (i = 1; i <= Counter[k]; i++) {
					if (Route[i][k] == SortII[j]) {
						SelectedStation = i;
						SelectedRoute = k;
						RemoveStation(SelectedRoute, SelectedStation);
					}
				}
			}
		}
	}

}

void LeastUsedStationRemoval3() {
	int i, j, k, l, m, NoRemovalStations, SelectedStation, SelectedRoute, NoUsedStations, NoRemovalS;
	float* EnergyQuantity, PercentRemovalS;
	NoUsedStations = 0;
	PercentRemovalS = 0.4;

	for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
		NoUsedStations += CounterStation[i];
	}
	if (NoUsedStations > 0) {
		// Select the stations to be removed
		if (NoUsedStations < NoStations) {
			NoRemovalS = floor(PercentRemovalS * NoUsedStations);
		}
		else {
			NoRemovalS = floor(PercentRemovalS * NoStations);
		}
		// Sort stations based on their number of utilization
		EnergyQuantity = new float[NoVertices + 1];
		for (i = 0; i <= NoVertices; i++) {
			EnergyQuantity[i] = 0;
		}
		for (j = NoCustomers + 1; j <= NoCustomers + NoStations; j++) {
			for (k = 1; k <= NoRoutes; k++) {
				for (i = 1; i <= Counter[k]; i++) {
					if (Route[i][k] == j) {
						for (m = 1; m <= NoChargers; m++) {
							EnergyQuantity[j] += vPtr[i][m][k];
						}
					}
				}
			}
		}
		// Sort stations based on their total distance
		for (i = NoCustomers + 1; i <= NoCustomers + NoStations - 1; i++) {
			for (j = i + 1; j <= NoCustomers + NoStations; j++) {
				if (EnergyQuantity[SortII[i]] > EnergyQuantity[SortII[j]]) {
					int temp;
					temp = SortII[j];
					for (l = j - 1; l >= i; l--) {
						SortII[l + 1] = SortII[l];
					}
					SortII[i] = temp;
				}
			}
		}
		// Remove the stations from the routes
		for (j = NoCustomers + NoStations - NoRemovalS; j < NoCustomers + NoStations; j++) {
			for (k = 1; k <= NoRoutes; k++) {
				for (i = 1; i <= Counter[k]; i++) {
					if (Route[i][k] == SortII[j]) {
						SelectedStation = i;
						SelectedRoute = k;
						RemoveStation(SelectedRoute, SelectedStation);
					}
				}
			}
		}
	}
}

void ExpensiveStationRemoval4() {
	int i, j, k, l, SelectedStation, SelectedRoute, NoUsedStations, NoRemovalS;
	float PercentRemovalS;
	NoUsedStations = 0;
	PercentRemovalS = 0.4;
	for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
		NoUsedStations += CounterStation[i];
	}
	if (NoUsedStations > 0) {
		// Determine number of stations to be removed
		if (NoUsedStations < NoStations) {
			NoRemovalS = ceil(PercentRemovalS * NoUsedStations);
		}
		else {
			NoRemovalS = ceil(PercentRemovalS * NoStations);
		}
		//Sort stations based on their technology
		for (i = NoCustomers + 1; i <= NoCustomers + NoStations - 1; i++) {
			for (j = i + 1; j <= NoCustomers + NoStations; j++) {
				if (Technology[SortII[i]] > Technology[SortII[j]]) {
					int temp;
					temp = SortII[j];
					for (l = j - 1; l >= i; l--) {
						SortII[l + 1] = SortII[l];
					}
					SortII[i] = temp;
				}
			}
		}
		// Remove the stations from the routes
		for (j = NoCustomers + NoStations; j > NoCustomers + NoStations - NoRemovalS; j--) {
			for (k = 1; k <= NoRoutes; k++) {
				for (i = 1; i <= Counter[k]; i++) {
					if (SortII[j] == Route[i][k]) {
						SelectedRoute = k;
						SelectedStation = i;
						RemoveStation(SelectedRoute, SelectedStation);
					}
				}
			}
		}
	}
}

void CustomerInsertion(int Variable1, int Variable2, int Variable3) {
	int i, j, k, l, m, CounterSpecial;

	CounterSpecial = 0;

	if (MaxEnergyCapacity[Variable1] < EnergyLimit) {
		Cost += RCost[1] * yPtr[Counter[Variable1]][Variable1];
	}
	for (i = Variable2; i <= Counter[Variable1]; i++) {
		for (m = 1; m <= NoChargers; m++) {
			Cost -= vPtr[i][m][Variable1] * RCost[m];
		}
	}
	float LengthDifference = d[Route[Variable2][Variable1]][RemovedCustomer[Variable3]] + d[RemovedCustomer[Variable3]][Route[Variable2 + 1][Variable1]] - d[Route[Variable2][Variable1]][Route[Variable2 + 1][Variable1]];
	RouteLength[Variable1] += LengthDifference;
	Cost += (TCost[Variable1] * (LengthDifference));
	//Emissions += (Lambda * CurbWeight * Rho * Sigma2nd * (LengthDifference));
	//Emissions += (Lambda * EngineFriction * EngineSpeed * EngineDisplacement * (LengthDifference/speed));
	//Emissions += (Lambda * Epsilon2nd * Rho * LengthDifference * speed * speed);
	//Emissions += (Rho * Sigma2nd * Lambda * ( qPtr[Variable2][Variable1] * d[Route[Variable2][Variable1]][RemovedCustomer[Variable3]] + (qPtr[Variable2][Variable1] - Demand[RemovedCustomer[Variable3]]) * d[RemovedCustomer[Variable3]][Route[Variable2+1][Variable1]] - qPtr[Variable2][Variable1] * d[Route[Variable2][Variable1]][Route[Variable2 + 1][Variable1]]));
	//for (i = Variable2 + 1; i <= Counter[Variable1]-1; i++) {
	//	Emissions += Rho * Sigma2nd * Lambda * d[Route[i][Variable1]][Route[i+1][Variable1]] * Demand[RemovedCustomer[Variable3]];
	//}
	//Cost = Cost + d[Route[Variable2][Variable1]][Route[Variable2+1][Variable1]] + d[Route[Variable2+1][Variable1]][Route[Variable2+2][Variable1]] - d[Route[Variable2][Variable1]][Route[Variable2+2][Variable1]];
	for (i = Variable2 + 1; i < Counter[Variable1]; i++) {
		if (Route[i][Variable1] > NoCustomers) {
			CounterSpecial = i;
			break;
		}
	}
	if (CounterSpecial > 0) {
		for (j = Counter[Variable1]; j > CounterSpecial; j--) {
			yPtr[j + 1][Variable1] = yPtr[j][Variable1];
			if (Route[j][Variable1] > NoCustomers&& Route[j][Variable1] < NoVertices) {
				vPtr[j + 1][Technology[Route[j][Variable1]]][Variable1] = vPtr[j][Technology[Route[j][Variable1]]][Variable1];
				vPtr[j][Technology[Route[j][Variable1]]][Variable1] = 0;
			}
		}
		for (j = CounterSpecial; j >= Variable2 + 1; j--) {
			yPtr[j + 1][Variable1] = yPtr[j][Variable1] - (EnergyConsumption[Variable1] * (d[Route[Variable2][Variable1]][RemovedCustomer[Variable3]] + d[RemovedCustomer[Variable3]][Route[Variable2 + 1][Variable1]] - d[Route[Variable2][Variable1]][Route[Variable2 + 1][Variable1]]));
		}
		yPtr[Variable2 + 1][Variable1] = yPtr[Variable2 + 1][Variable1] + EnergyConsumption[Variable1] * d[Route[Variable2][Variable1]][Route[Variable2 + 1][Variable1]] - EnergyConsumption[Variable1] * d[Route[Variable2][Variable1]][RemovedCustomer[Variable3]];
		if (Technology[Route[CounterSpecial][Variable1]] > 0 && Technology[Route[CounterSpecial][Variable1]] <= 3) {    //  Update Energy for the next station
			vPtr[CounterSpecial][Technology[Route[CounterSpecial][Variable1]]][Variable1] = 0;
			if (yPtr[CounterSpecial + 1][Variable1] < EnergyLimit) {
				if (yPtr[CounterSpecial + 1][Variable1] > 0) {
					vPtr[CounterSpecial + 1][Technology[Route[CounterSpecial][Variable1]]][Variable1] = 0.8 * MaxEnergyCapacity[Variable1] - yPtr[CounterSpecial + 1][Variable1];
				}
				else {
					vPtr[CounterSpecial + 1][Technology[Route[CounterSpecial][Variable1]]][Variable1] = 0.8 * MaxEnergyCapacity[Variable1];
				}
			}
		}
	}
	else {
		if (Counter[Variable1] == 1) {
			yPtr[0][Variable1] = Ratio * MaxEnergyCapacity[Variable1];
			yPtr[Counter[Variable1]][Variable1] = Ratio * MaxEnergyCapacity[Variable1];
		}
		for (j = Counter[Variable1]; j >= Variable2 + 1; j--) {
			yPtr[j + 1][Variable1] = yPtr[j][Variable1] - (EnergyConsumption[Variable1] * (d[Route[Variable2][Variable1]][RemovedCustomer[Variable3]] + d[RemovedCustomer[Variable3]][Route[Variable2 + 1][Variable1]] - d[Route[Variable2][Variable1]][Route[Variable2 + 1][Variable1]]));
		}
		yPtr[Variable2 + 1][Variable1] = yPtr[Variable2 + 1][Variable1] + EnergyConsumption[Variable1] * d[Route[Variable2][Variable1]][Route[Variable2 + 1][Variable1]] - EnergyConsumption[Variable1] * d[Route[Variable2][Variable1]][RemovedCustomer[Variable3]];
	}
	uPtr[Variable2 + 1][Variable1] = uPtr[Variable2 + 1][Variable1] + t[Route[Variable2][Variable1]][RemovedCustomer[Variable3]] - t[Route[Variable2][Variable1]][Route[Variable2 + 1][Variable1]];
	if (eTime[RemovedCustomer[Variable3]] <= uPtr[Variable2 + 1][Variable1]) {
		uPtr[Variable2 + 2][Variable1] = uPtr[Variable2 + 1][Variable1] + ServiceTime[RemovedCustomer[Variable3]] + t[RemovedCustomer[Variable3]][Route[Variable2 + 1][Variable1]];
	}
	else {
		uPtr[Variable2 + 2][Variable1] = eTime[RemovedCustomer[Variable3]] + ServiceTime[RemovedCustomer[Variable3]] + t[RemovedCustomer[Variable3]][Route[Variable2 + 1][Variable1]];
	}
	if (Counter[Variable1] == 1) {
		qPtr[0][Variable1] = MaxCapacity[Variable1];
		qPtr[Counter[Variable1]][Variable1] = qPtr[0][Variable1];
	}

	for (i = Counter[Variable1]; i > Variable2; i--) {
		qPtr[i + 1][Variable1] = qPtr[i][Variable1] - Demand[RemovedCustomer[Variable3]];
	}
	for (i = Variable2 + 2; i <= Counter[Variable1]; i++) {
		if (eTime[Route[i - 1][Variable1]] <= uPtr[i][Variable1]) {
			if (Route[i - 1][Variable1] > NoCustomers) { ////  Check if the previous node is a station or a customer 
				if (Technology[Route[i - 1][Variable1]] > 0 && Technology[Route[i - 1][Variable1]] <= 3) {    //  Update Time for the station Yes
					uPtr[i + 1][Variable1] = uPtr[i][Variable1] + vPtr[i][Technology[Route[i - 1][Variable1]]][Variable1] * Rate[Route[i - 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i][Variable1]];
				}
			}
			else {
				uPtr[i + 1][Variable1] = uPtr[i][Variable1] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i][Variable1]];
			}
		}
		else {
			if (Route[i - 1][Variable1] > NoCustomers) { ////  Check if the previous node is a station or a customer 
				if (Technology[Route[i - 1][Variable1]] > 0 && Technology[Route[i - 1][Variable1]] <= 3) {    //  Update Time for the station Yes
					uPtr[i + 1][Variable1] = eTime[Route[i - 1][Variable1]] + vPtr[i][Technology[Route[i - 1][Variable1]]][Variable1] * Rate[Route[i - 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i][Variable1]];
				}
			}
			else {
				uPtr[i + 1][Variable1] = eTime[Route[i - 1][Variable1]] + ServiceTime[Route[i - 1][Variable1]] + t[Route[i - 1][Variable1]][Route[i][Variable1]];
			}
		}
	}
	for (i = Counter[Variable1]; i > Variable2; i--) {
		Route[i + 1][Variable1] = Route[i][Variable1];
	}
	Route[Variable2 + 1][Variable1] = RemovedCustomer[Variable3];
	Counter[Variable1]++;
	RoutedCustomers++;
	for (i = Variable2; i <= Counter[Variable1]; i++) {
		for (m = 1; m <= NoChargers; m++) {
			Cost += vPtr[i][m][Variable1] * RCost[m];
		}
	}
	if (MaxEnergyCapacity[Variable1] < EnergyLimit) {
		Cost -= RCost[1] * yPtr[Counter[Variable1]][Variable1];
	}
	for (j = Variable3; j <= CounterRemoval; j++) {
		RemovedCustomer[j] = RemovedCustomer[j + 1];
	}
	RemovedCustomer[CounterRemoval] = 0;
	CounterRemoval--;
}

void CheckTimeWindowFunction(int Variable1, int Variable2, int Variable3) {
	int i, j, k, l, CounterSpecial;
	CounterSpecial = 0;

	TimeWindowCheck = 0;
	for (i = Variable1 + 1; i < Counter[Variable3]; i++) {
		if (Route[i][Variable3] > NoCustomers) {
			CounterSpecial = i;
			break;
		}
	}
	if (CounterSpecial > 0) {
		for (j = Counter[Variable3]; j > CounterSpecial; j--) {
			TempY[j + 1][Variable3] = yPtr[j][Variable3];
			if (Route[j][Variable3] > NoCustomers&& Route[j][Variable3] < NoVertices) {
				TempV[j + 1][Technology[Route[j][Variable3]]][Variable3] = vPtr[j][Technology[Route[j][Variable3]]][Variable3];
			}
		}
		for (j = CounterSpecial; j >= Variable1 + 1; j--) {
			TempY[j + 1][Variable3] = yPtr[j][Variable3] - (EnergyConsumption[Variable3] * (d[Route[Variable1][Variable3]][RemovedCustomer[Variable2]] + d[RemovedCustomer[Variable2]][Route[Variable1 + 1][Variable3]] - d[Route[Variable1][Variable3]][Route[Variable1 + 1][Variable3]]));
		}
		TempY[Variable1 + 1][Variable3] = yPtr[Variable1][Variable3] - EnergyConsumption[Variable3] * d[Route[Variable1][Variable3]][RemovedCustomer[Variable3]];
		if (Technology[Route[CounterSpecial][Variable3]] > 0 && Technology[Route[CounterSpecial][Variable3]] <= 3) {    //  Update Energy for the next station
			TempV[CounterSpecial][Technology[Route[CounterSpecial][Variable3]]][Variable3] = 0;
			if (TempY[CounterSpecial + 1][Variable3] < EnergyLimit) {
				if (TempY[CounterSpecial + 1][Variable3] > 0) {
					TempV[CounterSpecial + 1][Technology[Route[CounterSpecial][Variable3]]][Variable3] = 0.8 * MaxEnergyCapacity[Variable3] - TempY[CounterSpecial + 1][Variable3];
				}
				else {
					TempV[CounterSpecial + 1][Technology[Route[CounterSpecial][Variable3]]][Variable3] = 0.8 * MaxEnergyCapacity[Variable3];
				}
			}
		}
	}
	TempU[Variable1 + 1] = uPtr[Variable1 + 1][Variable3] + t[Route[Variable1][Variable3]][RemovedCustomer[Variable2]] - t[Route[Variable1][Variable3]][Route[Variable1 + 1][Variable3]];
	if (TempU[Variable1 + 1] > lTime[RemovedCustomer[Variable2]]) {
		TimeWindowCheck = 1;
	}
	if (eTime[RemovedCustomer[Variable2]] <= TempU[Variable1 + 1]) {
		TempU[Variable1 + 2] = TempU[Variable1 + 1] + ServiceTime[RemovedCustomer[Variable2]] + t[RemovedCustomer[Variable2]][Route[Variable1 + 1][Variable3]];
	}
	else {
		TempU[Variable1 + 2] = eTime[RemovedCustomer[Variable2]] + ServiceTime[RemovedCustomer[Variable2]] + t[RemovedCustomer[Variable2]][Route[Variable1 + 1][Variable3]];
	}

	if (TempU[Variable1 + 2] > lTime[Route[Variable1 + 1][Variable3]]) {
		TimeWindowCheck = 1;
	}
	for (l = Variable1 + 2; l <= Counter[Variable3]; l++) {
		if (eTime[Route[l - 1][Variable3]] <= TempU[l]) {
			if (Route[l - 1][Variable3] > NoCustomers) { ////  Check if the previous node is a station or a customer 
				if (Technology[Route[l - 1][Variable3]] > 0 && Technology[Route[l - 1][Variable3]] <= 3) {    //  Update Time for the station Yes
					TempU[l + 1] = TempU[l] + TempV[l][Technology[Route[l - 1][Variable3]]][Variable3] * Rate[Route[l - 1][Variable3]] + ServiceTime[Route[l - 1][Variable3]] + t[Route[l - 1][Variable3]][Route[l][Variable3]];
				}
			}
			else {
				TempU[l + 1] = TempU[l] + ServiceTime[Route[l - 1][Variable3]] + t[Route[l - 1][Variable3]][Route[l][Variable3]];
			}
		}
		else {
			if (Route[l - 1][Variable3] > NoCustomers) { ////  Check if the previous node is a station or a customer 
				if (Technology[Route[l - 1][Variable3]] > 0 && Technology[Route[l - 1][Variable3]] <= 3) {    //  Update Time for the station Yes
					TempU[l + 1] = eTime[Route[l - 1][Variable3]] + TempV[l][Technology[Route[l - 1][Variable3]]][Variable3] * Rate[Route[l - 1][Variable3]] + ServiceTime[Route[l - 1][Variable3]] + t[Route[l - 1][Variable3]][Route[l][Variable3]];
				}
			}
			else {
				TempU[l + 1] = eTime[Route[l - 1][Variable3]] + ServiceTime[Route[l - 1][Variable3]] + t[Route[l - 1][Variable3]][Route[l][Variable3]];
			}
		}
		if (TempU[l + 1] > lTime[Route[l][Variable3]]) {
			TimeWindowCheck = 1;
		}
	}
}

void SortRoutes(int Variable1) {
	int i, j, k;
	float Cost2, Cost4, Cost1, Cost3;

	for (j = 1; j <= Variable1; j++) {
		if (Counter[j] == 0) {
			Cost1 = MaxEnergyCapacity[j];
			Cost2 = TCost[j];
			Cost3 = AcquisitionCost[j];
			Cost4 = EnergyConsumption[j];
			for (k = j + 1; k <= Variable1; k++) {
				for (i = 0; i <= Counter[k]; i++) {
					if (Route[i][k] > NoCustomers&& Route[i][k] < NoVertices) {
						vPtr[i][Technology[Route[i][k]]][k - 1] = vPtr[i][Technology[Route[i][k]]][k];
					}
					Route[i][k - 1] = Route[i][k];
					qPtr[i][k - 1] = qPtr[i][k];
					yPtr[i][k - 1] = yPtr[i][k];
					uPtr[i][k - 1] = uPtr[i][k];
				}
				if (Route[Counter[k] + 1][k - 1] != 0) {
					for (int l = Counter[k] + 1; l <= Counter[k - 1]; l++) {
						if (Route[l][k - 1] > NoCustomers&& Route[l][k - 1] < NoVertices) {
							vPtr[l][Technology[Route[l][k - 1]]][k - 1] = 0;
						}
						Route[l][k - 1] = 0;
						qPtr[l][k - 1] = 0;
						yPtr[l][k - 1] = 0;
						uPtr[l][k - 1] = 0;
					}
				}
				Counter[k - 1] = Counter[k];
				RouteLength[k - 1] = RouteLength[k];
				TCost[k - 1] = TCost[k];
				MaxEnergyCapacity[k - 1] = MaxEnergyCapacity[k];
				AcquisitionCost[k - 1] = AcquisitionCost[k];
				EnergyConsumption[k - 1] = EnergyConsumption[k];
			}
			for (i = 1; i <= Counter[Variable1]; i++) {
				if (Route[i][Variable1] > NoCustomers&& Route[i][Variable1] < NoVertices) {
					vPtr[i][Technology[Route[i][Variable1]]][Variable1] = 0;
				}
				Route[i][Variable1] = 0;
				qPtr[i][Variable1] = 0;
				yPtr[i][Variable1] = 0;
				uPtr[i][Variable1] = 0;
			}
			Counter[Variable1] = 0;
			RouteLength[Variable1] = 0;
			MaxEnergyCapacity[Variable1] = Cost1;
			TCost[Variable1] = Cost2;
			AcquisitionCost[Variable1] = Cost3;
			EnergyConsumption[Variable1] = Cost4;
			Variable1--;
			j = 0;
		}
	}
}

void StationInsertion(int Variable1, int Variable2) {
	int i, j, k, l, m, CounterSpecial, CounterUsedStation;
	CounterUsedStation = 1;

label3:
	for (i = 0; i <= NoVertices ; i++) {
		for (m = 0; m <= NoChargers; m++) {
			for (k = 0; k <= NoVehicles; k++) {
				TempV[i][m][k] = 0;
			}
		}
	}
	for (i = 0; i <= Counter[Variable2]; i++) {
		for (m = 1; m <= NoChargers; m++) {
			TempV[i][m][Variable2] = vPtr[i][m][Variable2];
		}
	}

	CounterSpecial = 0;
	// Update Energy
	if (Route[Variable1 - 1][Variable2] > NoCustomers) {
		TempY[Variable1][Variable2] = 0.8 * MaxEnergyCapacity[Variable2] - EnergyConsumption[Variable2] * d[Route[Variable1 - 1][Variable2]][SortII[NoCustomers + 1]];
	}
	else {
		TempY[Variable1][Variable2] = yPtr[Variable1 - 1][Variable2] - EnergyConsumption[Variable2] * d[Route[Variable1 - 1][Variable2]][SortII[NoCustomers + 1]];
	}
	if (TempY[Variable1][Variable2] < EnergyLimit) {
		if (TempY[Variable1][Variable2] > 0) {
			if (Technology[SortII[NoCustomers + 1]] > 0 && Technology[SortII[NoCustomers + 1]] <= 3) {
				TempV[Variable1][Technology[SortII[NoCustomers + 1]]][Variable2] = 0.8 * MaxEnergyCapacity[Variable2] - TempY[Variable1][Variable2];
			}
		}
		else {
			TempV[Variable1][Technology[SortII[NoCustomers + 1]]][Variable2] = 0.8 * MaxEnergyCapacity[Variable2];
		}
	}
	// Find the next station
	for (j = Variable1; j <= Counter[Variable2] - 1; j++) {
		if (Route[j][Variable2] > NoCustomers) {
			CounterSpecial = j;
			break;
		}
	}
	// Update Energy for the successor nodes after the next station
	if (CounterSpecial > 0) {
		for (j = Counter[Variable2]; j > CounterSpecial; j--) {
			TempY[j + 1][Variable2] = yPtr[j][Variable2];
		}
		for (j = Variable1; j <= CounterSpecial; j++) {
			if (Technology[SortII[NoCustomers + 1]] > 0 && Technology[SortII[NoCustomers + 1]] <= 3) {
				TempY[j + 1][Variable2] = yPtr[j][Variable2] + TempV[Variable1][Technology[SortII[NoCustomers + 1]]][Variable2] - EnergyConsumption[Variable2] * d[Route[Variable1 - 1][Variable2]][SortII[NoCustomers + 1]] - EnergyConsumption[Variable2] * d[SortII[NoCustomers + 1]][Route[Variable1][Variable2]] + EnergyConsumption[Variable2] * d[Route[Variable1 - 1][Variable2]][Route[Variable1][Variable2]];
			}
			else {
				TempY[j + 1][Variable2] = yPtr[j][Variable2] + 0.8 * MaxEnergyCapacity[Variable2] - EnergyConsumption[Variable2] * d[Route[Variable1 - 1][Variable2]][SortII[NoCustomers + 1]] - EnergyConsumption[Variable2] * d[SortII[NoCustomers + 1]][Route[Variable1][Variable2]] + EnergyConsumption[Variable2] * d[Route[Variable1 - 1][Variable2]][Route[Variable1][Variable2]];
			}
		}
		if (Variable1 != CounterSpecial) {
			TempV[CounterSpecial][Technology[Route[CounterSpecial][Variable2]]][Variable2] = 0;
		}
		if (TempY[CounterSpecial + 1][Variable2] < EnergyLimit) {
			if (TempY[CounterSpecial + 1][Variable2] > 0) {
				TempV[CounterSpecial + 1][Technology[Route[CounterSpecial][Variable2]]][Variable2] = 0.8 * MaxEnergyCapacity[Variable2] - TempY[CounterSpecial + 1][Variable2];
			}
			else {
				TempV[CounterSpecial + 1][Technology[Route[CounterSpecial][Variable2]]][Variable2] = 0.8 * MaxEnergyCapacity[Variable2];
			}
		}
		//Update load of Energy for the rest of the stations
		for (j = CounterSpecial + 1; j <= Counter[Variable2]; j++) {
			if (Route[j][Variable2] > NoCustomers&& Route[j][Variable2] < NoVertices) {
				TempV[j + 1][Technology[Route[j][Variable2]]][Variable2] = vPtr[j][Technology[Route[j][Variable2]]][Variable2];
			}
		}

	}
	// Update Energy for the rest of the nodes if the station is the last station
	else {
		for (j = Variable1; j <= Counter[Variable2]; j++) {
			if (Technology[SortII[NoCustomers + 1]] > 0 && Technology[SortII[NoCustomers + 1]] <= 3) {
				TempY[j + 1][Variable2] = yPtr[j][Variable2] + TempV[Variable1][Technology[SortII[NoCustomers + 1]]][Variable2] - EnergyConsumption[Variable2] * d[Route[Variable1 - 1][Variable2]][SortII[NoCustomers + 1]] - EnergyConsumption[Variable2] * d[SortII[NoCustomers + 1]][Route[Variable1][Variable2]] + EnergyConsumption[Variable2] * d[Route[Variable1 - 1][Variable2]][Route[Variable1][Variable2]];
			}
		}
	}

	TimeWindowCheck = 0;
	// Update time for the next node after the inserted station
	if (eTime[Route[Variable1 - 1][Variable2]] <= uPtr[Variable1 - 1][Variable2]) {
		TempU[Variable1] = uPtr[Variable1 - 1][Variable2] + ServiceTime[Route[Variable1 - 1][Variable2]] + t[Route[Variable1 - 1][Variable2]][SortII[NoCustomers + 1]];
	}
	else {
		TempU[Variable1] = eTime[Route[Variable1 - 1][Variable2]] + ServiceTime[Route[Variable1 - 1][Variable2]] + t[Route[Variable1 - 1][Variable2]][SortII[NoCustomers + 1]];
	}
	if (eTime[SortII[NoCustomers + 1]] <= TempU[Variable1]) {
		if (Technology[SortII[NoCustomers + 1]] > 0 && Technology[SortII[NoCustomers + 1]] <= 3) {
			TempU[Variable1 + 1] = TempU[Variable1] + TempV[Variable1][Technology[SortII[NoCustomers + 1]]][Variable2] * Rate[SortII[NoCustomers + 1]] + ServiceTime[SortII[NoCustomers + 1]] + t[SortII[NoCustomers + 1]][Route[Variable1][Variable2]];
		}
	}
	else {
		if (Technology[SortII[NoCustomers + 1]] > 0 && Technology[SortII[NoCustomers + 1]] <= 3) {
			TempU[Variable1 + 1] = eTime[SortII[NoCustomers + 1]] + TempV[Variable1][Technology[SortII[NoCustomers + 1]]][Variable2] * Rate[SortII[NoCustomers + 1]] + ServiceTime[SortII[NoCustomers + 1]] + t[SortII[NoCustomers + 1]][Route[Variable1][Variable2]];
		}
	}
	if (TempU[Variable1 + 1] > lTime[Route[Variable1][Variable2]]) {
		TimeWindowCheck = 1;
	}
	/// Take care of the nodes after the station
	for (l = Variable1 + 2; l <= Counter[Variable2] + 1; l++) {
		if (Route[l - 2][Variable2] > NoCustomers) {
			if (eTime[Route[l - 2][Variable2]] <= TempU[l - 1]) {
				if (Technology[Route[l - 2][Variable2]] > 0 && Technology[Route[l - 2][Variable2]] <= 3) {
					TempU[l] = TempU[l - 1] + TempV[l - 1][Technology[Route[l - 2][Variable2]]][Variable2] * Rate[Route[l - 2][Variable2]] + ServiceTime[Route[l - 2][Variable2]] + t[Route[l - 2][Variable2]][Route[l - 1][Variable2]];
				}
			}
			else {
				if (Technology[Route[l - 2][Variable2]] > 0 && Technology[Route[l - 2][Variable2]] <= 3) { ///???
					TempU[l] = eTime[Route[l - 2][Variable2]] + TempV[l - 1][Technology[Route[l - 2][Variable2]]][Variable2] * Rate[Route[l - 2][Variable2]] + ServiceTime[Route[l - 2][Variable2]] + t[Route[l - 2][Variable2]][Route[l - 1][Variable2]];
				}
			}
			if (TempU[l] > lTime[Route[l - 1][Variable2]]) {
				TimeWindowCheck = 1;
			}
		}
		else {
			if (eTime[Route[l - 2][Variable2]] <= TempU[l - 1]) {
				TempU[l] = TempU[l - 1] + ServiceTime[Route[l - 2][Variable2]] + t[Route[l - 2][Variable2]][Route[l - 1][Variable2]];
			}
			else {
				TempU[l] = eTime[Route[l - 2][Variable2]] + ServiceTime[Route[l - 2][Variable2]] + t[Route[l - 2][Variable2]][Route[l - 1][Variable2]];
			}
		}
		if (TempU[l] > lTime[Route[l - 1][Variable2]]) {
			TimeWindowCheck = 1;
		}
	}
	if (TempU[Variable1] > lTime[SortII[NoCustomers + 1]]) {
		TimeWindowCheck = 1;
	}
	if (TimeWindowCheck == 0) {
		for (i = Variable1; i <= Counter[Variable2]; i++) {
			for (m = 1; m <= NoChargers; m++) {
				Cost -= vPtr[i][m][Variable2] * RCost[m];
			}
		}
		if (Variable1 != CounterSpecial) {
			if (Route[CounterSpecial][Variable2] > NoCustomers&& Route[CounterSpecial][Variable2] < NoVertices) {
				vPtr[CounterSpecial][Technology[Route[CounterSpecial][Variable2]]][Variable2] = 0;
			}
		}
		if (MaxEnergyCapacity[Variable2] < EnergyLimit) {
			Cost += RCost[1] * yPtr[Counter[Variable2]][Variable2];
		}
		// Update Load and Time
		for (i = Variable1; i <= Counter[Variable2]; i++) {
			for (m = 1; m <= NoChargers; m++) {
				vPtr[i][m][Variable2] = 0;
			}
		}
		vPtr[Variable1][Technology[SortII[NoCustomers + 1]]][Variable2] = TempV[Variable1][Technology[SortII[NoCustomers + 1]]][Variable2];
		for (j = Counter[Variable2]; j >= Variable1; j--) {
			qPtr[j + 1][Variable2] = qPtr[j][Variable2];
			uPtr[j + 1][Variable2] = TempU[j + 1];
			yPtr[j + 1][Variable2] = TempY[j + 1][Variable2];
			if (Route[j][Variable2] > NoCustomers&& Route[j][Variable2] < NoVertices) {
				vPtr[j + 1][Technology[Route[j][Variable2]]][Variable2] = TempV[j + 1][Technology[Route[j][Variable2]]][Variable2];
			}
		}
		yPtr[Variable1][Variable2] = TempY[Variable1][Variable2];
		uPtr[Variable1][Variable2] = TempU[Variable1];
		// Update Cost and Length
		float LengthDifference = d[Route[Variable1 - 1][Variable2]][SortII[NoCustomers + 1]] + d[SortII[NoCustomers + 1]][Route[Variable1][Variable2]] - d[Route[Variable1 - 1][Variable2]][Route[Variable1][Variable2]];
		RouteLength[Variable2] += LengthDifference;
		for (i = Variable1; i <= Counter[Variable2]; i++) {
			for (m = 1; m <= NoChargers; m++) {
				Cost += vPtr[i][m][Variable2] * RCost[m];
			}
		}
		Cost += (TCost[Variable2] * LengthDifference);
		//Cost = Cost + d[Route[Variable1-1][Variable2]][SortII[NoCustomers+1]] + d[SortII[NoCustomers+1]][Route[Variable1][Variable2]] - d[Route[Variable1-1][Variable2]][Route[Variable1][Variable2]];
		for (j = Counter[Variable2]; j >= Variable1; j--) {
			Route[j + 1][Variable2] = Route[j][Variable2];
		}
		Route[Variable1][Variable2] = SortII[NoCustomers + 1];
		CounterStation[SortII[NoCustomers + 1]]++;
		Counter[Variable2]++;
		if (MaxEnergyCapacity[Variable2] < EnergyLimit) {
			Cost -= RCost[1] * yPtr[Counter[Variable2]][Variable2];
		}
	}
	else {
		if (CounterUsedStation < NoStations) {
			CounterUsedStation++;
			int temp = SortII[NoCustomers + 1];
			for (j = NoCustomers + 1; j <= NoCustomers + NoStations - 1; j++) {
				SortII[j] = SortII[j + 1];
			}
			if (SortII[NoCustomers + 1] == Route[Variable1 - 1][Variable2]) {
				int temp2 = SortII[NoCustomers + 2];
				SortII[NoCustomers + 2] = SortII[NoCustomers + 1];
				SortII[NoCustomers + 1] = temp2;
			}
			SortII[NoCustomers + NoStations] = temp;
			goto label3;
		}
	}
}

void GreedyStationInsertion1() {
	int i, j, k, CounterSpecial;
	TimeWindowCheck = 0;

	for (k = 1; k <= NoRoutes; k++) {
		for (i = 1; i <= Counter[k]; i++) {
			if (yPtr[i][k] < 0) {
			label4:
				// If previous node is a station, it is not reasonable to have two stations in a row
				if (Route[i - 1][k] > NoCustomers || (Route[i][k] > NoCustomers&& Route[i][k] < NoVertices)) {
					break;
				}
				else {
					// Find best station to insert between the customer with negative energy level and its previous node which have to be another customer
					SortStations(Route[i - 1][k], Route[i][k]);
					if (yPtr[i - 1][k] >= EnergyConsumption[k] * d[Route[i - 1][k]][SortII[NoCustomers + 1]]) {
						StationInsertion(i, k);
						if (TimeWindowCheck == 1) {
							break;
						}
					}
					else {
						i = i - 1;
						if (i < 1) {
							break;
						}
						goto label4;
					}
				}
			}
		}
	}
}

void GreedyStationInsertionComparison2() {
	int i, j, k, CounterSpecial;
	float tempLength1, tempLength2;
	TimeWindowCheck = 0;
	for (k = 1; k <= NoRoutes; k++) {
		for (i = 2; i <= Counter[k]; i++) {
			if (yPtr[i][k] < 0) {
			lablel4:
				if (Route[i - 2][k] > NoCustomers || Route[i - 1][k] > NoCustomers || (Route[i][k] > NoCustomers&& Route[i][k] < NoVertices)) {
					break;
				}
				else {
					// Find station between two previous nodes
					SortStations(Route[i - 2][k], Route[i - 1][k]);
					tempLength1 = BigNumber;
					tempLength2 = BigNumber;
					if (yPtr[i - 2][k] >= EnergyConsumption[k] * d[Route[i - 2][k]][SortII[NoCustomers + 1]]) {
						tempLength1 = RouteLength[k] + d[Route[i - 2][k]][SortII[NoCustomers + 1]] + d[SortII[NoCustomers + 1]][Route[i - 1][k]] - d[Route[i - 2][k]][Route[i - 1][k]];
						// Find station between the customer and the previous node
						SortStations(Route[i - 1][k], Route[i][k]);
						if (yPtr[i - 1][k] >= EnergyConsumption[k] * d[Route[i - 1][k]][SortII[NoCustomers + 1]]) {
							tempLength2 = RouteLength[k] + d[Route[i - 1][k]][SortII[NoCustomers + 1]] + d[SortII[NoCustomers + 1]][Route[i][k]] - d[Route[i - 1][k]][Route[i][k]];
						}
						if (tempLength1 == BigNumber && tempLength2 == BigNumber) {
							break;
						}
						if (tempLength2 < tempLength1) {
							SortStations(Route[i - 1][k], Route[i][k]);
							StationInsertion(i, k);
							if (TimeWindowCheck == 1 && tempLength1 != BigNumber) {
								SortStations(Route[i - 2][k], Route[i - 1][k]);
								StationInsertion(i - 1, k);
								if (TimeWindowCheck == 1) {
									break;
								}
							}
						}
						else {
							SortStations(Route[i - 2][k], Route[i - 1][k]);
							StationInsertion(i - 1, k);
							if (TimeWindowCheck == 1 && tempLength2 != BigNumber) {
								SortStations(Route[i - 1][k], Route[i][k]);
								StationInsertion(i, k);
								if (TimeWindowCheck == 1) {
									break;
								}
							}
						}
					}
					else {
						SortStations(Route[i - 1][k], Route[i][k]);
						if (i > 2 && yPtr[i - 1][k] >= EnergyConsumption[k] * d[Route[i - 1][k]][SortII[NoCustomers + 1]]) {
							i = i - 1;
							goto lablel4;
						}
						else {
							break;
						}
					}
				}
			}
		}
	}
}

void BestStationInsertion3() {
	/*int i, j, k, l, CounterSpecial, SelectedCustomer;
	float Min, Distance, tempY1;
	SelectedCustomer = 0;

	for( k = 1; k <= NoRoutes; k++){
		for( i = 1; i <= Counter[k]; i++){
			if(yPtr[i][k] < 0){
				if( Route[i-1][k] > NoCustomers ){
					break;
				}
				else{
					for( j = i - 1; j >= 0; j--){
						if((Route[j][k] > NoCustomers && Route[j][k] < NoVertices) || ( Route[j][k] == 0)){
							SelectedCustomer = 0;
							Min = BigNumber;
							tempY1 = 0;
							for( l = i; l > j; l--){
								SortStations(Route[l-1][k], Route[l][k]);
								if(yPtr[l-1][k] >= EnergyConsumption[k] * d[Route[l-1][k]][SortII[NoCustomers+1]] ){
									tempY1 = MaxEnergyCapacity[k] -d[SortII[NoCustomers+1]][Route[l][k]];
									for ( int a = l; a < i; a++){
										tempY1 -= d[Route[a][k]][Route[a+1][k]];
									}
									if( tempY1 > 0 ){
										Distance = RouteLength[k] + d[Route[l-1][k]][SortII[NoCustomers+1]] + d[SortII[NoCustomers+1]][Route[l][k]] - d[Route[l-1][k]][Route[l][k]];
										if( Distance < Min){
											Min = Distance;
											SelectedCustomer = l;
										}
									}
								}
							}
							if (SelectedCustomer > 0){
								SortStations(Route[SelectedCustomer-1][k], Route[SelectedCustomer][k]);
								StationInsertion(SelectedCustomer, k);
								j = -1;
							}
						}
					}
				}
			}
		}
	}*/
}

void GreedyCustomerInsertion1() {
	int i, j, k, l, m, SelectedRoute, SelectedCustomer, SelectedNode, CounterSpecial, NewRoute, AddPoint;
	float Distance, Min;
	int* Check;

	Check = new int[NoCustomers + 1];
	for (i = 0; i <= NoCustomers; i++) {
		Check[i] = 0;
	}

	for (j = 1; j <= CounterRemoval; j++) {
		if (Check[RemovedCustomer[j]] == 0) {
			NewRoute = 0;
			AddPoint = 0;
			Min = BigNumber;
			CounterSpecial = 0;
			Check[RemovedCustomer[j]] = 1;
			for (k = 1; k <= NoVehicles; k++) {  // InitialNoRoutes
				if (Counter[k] == 0) {
					Counter[k] = 1;
					Route[Counter[k]][k] = NoVertices;
					TempQ[Counter[k]][k] = qPtr[0][k];

				}
				for (i = 0; i <= Counter[k] - 1; i++) {
					if (Counter[k] == 1) {
						TempQ[Counter[k]][k] = qPtr[0][k];
					}
					else {
						TempQ[Counter[k]][k] = qPtr[Counter[k]][k] - Demand[RemovedCustomer[j]];  //  Update Load for the end depot
					}
					Distance = d[Route[i][k]][RemovedCustomer[j]] + d[RemovedCustomer[j]][Route[i + 1][k]] - d[Route[i][k]][Route[i + 1][k]];
					if (Distance < Min && TempQ[Counter[k]][k] >= 0) { /// Check for load capacity
						// Check for time window
						CheckTimeWindowFunction(i, j, k);
						if (TimeWindowCheck == 0) {
							AddPoint = 1;
							Min = Distance;
							SelectedRoute = k;
							SelectedCustomer = i;
							SelectedNode = j;
							//break;
					/*if (MaxEnergyCapacity[k] < EnergyLimit ){
						if( NoEV <= ceil (Epsilon * NoCV)  ){
							AddPoint = 1;
							Min = Distance;
							SelectedRoute = k;
							SelectedCustomer = i;
							SelectedNode = j;
							//break;
						}
					}
					else{
						AddPoint = 1;
						Min = Distance;
						SelectedRoute = k;
						SelectedCustomer = i;
						SelectedNode = j;
						//break;
					}*/
						}
					}
				}
			}
			if (AddPoint == 1) {
				for (int kk = 1; kk <= NoVehicles; kk++) { //InitialNoRoutes
					if (Counter[kk] == 1) {
						if (kk == SelectedRoute) {
							NewRoute = 1;
						}
						else {
							Route[Counter[kk]][kk] = 0;
							Counter[kk] = 0;
						}
					}
				}

				if (NewRoute == 1) {
					NoRoutes++;
					if (MaxEnergyCapacity[SelectedRoute] < EnergyLimit) {
						Cost += (RCost[1] * Ratio * MaxEnergyCapacity[SelectedRoute]);
						Cost += AcquisitionCost[SelectedRoute];
						NoEV++;
					}
					else {
						Cost += AcquisitionCost[SelectedRoute];
						NoCV++;
					}
				}
				CustomerInsertion(SelectedRoute, SelectedCustomer, SelectedNode);
				/*	int Infeasibility = 0;
					for ( k = 1; k <= NoRoutes; k++){
						for ( i = 1; i <= Counter[k]; i++ ){
							if ( yPtr[i][k] < 0){
								Infeasibility = 1;
								break;
							}
						}
					}
				if ( Infeasibility == 1){
					GreedyStationInsertion1();

					}*/

				if (NewRoute == 1) {
					SortRoutes(SelectedRoute);
				}
			}
			j = 0;
		}

	}

}

void RegretCustomerInsertion2() {
	int i, j, k, l, SelectedRoute, SelectedCustomer, SelectedNode, * SelectedRoute1, * SelectedCustomer1, * SelectedRoute2, * SelectedCustomer2, CounterSpecial, NewRoute, AddPoint, * RegretPossibility, Regret;
	float Distance, Min1, Min2, * CostDifference, Max;
	float TempCost1, TempCost2 = 0;
	CostDifference = new float[NoCustomers + 1];
	SelectedRoute1 = new int[NoVehicles + 1];
	SelectedCustomer1 = new int[NoCustomers + 1];
	SelectedRoute2 = new int[NoVehicles + 1];
	SelectedCustomer2 = new int[NoCustomers + 1];
	RegretPossibility = new int[NoCustomers + 1];
	SelectedRoute = SelectedCustomer = SelectedNode = 0;

	int* Check;

	Check = new int[NoCustomers + 1];
	for (i = 0; i <= NoCustomers; i++) {
		Check[i] = 0;
	}

	for (j = 1; j <= CounterRemoval; j++) {
		if (Check[RemovedCustomer[j]] == 0) {
			NewRoute = 0;
			AddPoint = 0;
			CounterSpecial = 0;
			Min1 = BigNumber;
			Min2 = BigNumber;
			Check[RemovedCustomer[j]] = 1;
			for (i = 0; i <= NoCustomers; i++) {
				CostDifference[i] = 0;
			}
			for (i = 0; i <= NoVehicles; i++) {
				SelectedRoute1[i] = 0;
			}
			for (i = 0; i <= NoCustomers; i++) {
				SelectedCustomer1[i] = 0;
			}
			for (i = 0; i <= NoVehicles; i++) {
				SelectedRoute2[i] = 0;
			}
			for (i = 0; i <= NoCustomers; i++) {
				SelectedCustomer2[i] = 0;
			}
			for (i = 0; i <= NoCustomers; i++) {
				RegretPossibility[i] = 0;
			}
			for (k = 1; k <= NoVehicles; k++) { // InitialNoRoutes
				if (Counter[k] == 0) {
					Counter[k] = 1;
					Route[Counter[k]][k] = NoVertices;
					TempQ[Counter[k]][k] = qPtr[0][k];
				}
				else {
					TempQ[Counter[k]][k] = qPtr[Counter[k]][k] - Demand[RemovedCustomer[j]];  //  Update Load for the end depot
				}
				for (i = 0; i <= Counter[k] - 1; i++) {
					Distance = d[Route[i][k]][RemovedCustomer[j]] + d[RemovedCustomer[j]][Route[i + 1][k]] - d[Route[i][k]][Route[i + 1][k]];
					if (Distance < Min2 && TempQ[Counter[k]][k] >= 0) { /// Check for load capacity
						// Check for time window
						CheckTimeWindowFunction(i, j, k);
						if (TimeWindowCheck == 0) {
							if (Distance < Min1 && TimeWindowCheck == 0) {
								AddPoint = 1;
								Min2 = Min1;
								Min1 = Distance;
								// Take care of selected ones
								SelectedRoute2[j] = SelectedRoute1[j];
								SelectedCustomer2[j] = SelectedCustomer1[j];
								SelectedRoute1[j] = k;
								SelectedCustomer1[j] = i;
							}
							else if (Distance < Min2 && TimeWindowCheck == 0) {
								Min2 = Distance;
								SelectedRoute2[j] = k;
								SelectedCustomer2[j] = i;
							}
						}
					}
				}
			}
			if (SelectedRoute1[j] > 0 && SelectedRoute2[j] > 0) {
				RegretPossibility[j] = 1;
				TempCost1 = Cost + RCost[1] * yPtr[Counter[SelectedRoute1[j]]][SelectedRoute1[j]];
				TempCost1 = Cost + (TCost[SelectedRoute1[j]] * (d[Route[SelectedCustomer1[j]][SelectedRoute1[j]]][RemovedCustomer[j]] + d[RemovedCustomer[j]][Route[SelectedCustomer1[j] + 1][SelectedRoute1[j]]] - d[Route[SelectedCustomer1[j]][SelectedRoute1[j]]][Route[SelectedCustomer1[j] + 1][SelectedRoute1[j]]]));
				//TempCost1 = Cost + d[Route[SelectedCustomer1[j]][SelectedRoute1[j]]][RemovedCustomer[j]] + d[RemovedCustomer[j]][Route[SelectedCustomer1[j]+1][SelectedRoute1[j]]] - d[Route[SelectedCustomer1[j]][SelectedRoute1[j]]][Route[SelectedCustomer1[j]+1][SelectedRoute1[j]]];
				TempCost1 = Cost - RCost[1] * yPtr[Counter[SelectedRoute1[j]]][SelectedRoute1[j]];
				TempCost2 = Cost + RCost[1] * yPtr[Counter[SelectedRoute2[j]]][SelectedRoute2[j]];
				//TempCost2 = Cost + d[Route[SelectedCustomer2[j]][SelectedRoute2[j]]][RemovedCustomer[j]] + d[RemovedCustomer[j]][Route[SelectedCustomer2[j]+1][SelectedRoute2[j]]] - d[Route[SelectedCustomer2[j]][SelectedRoute2[j]]][Route[SelectedCustomer2[j]+1][SelectedRoute2[j]]];
				TempCost2 = Cost + (TCost[SelectedRoute2[j]] * (d[Route[SelectedCustomer2[j]][SelectedRoute2[j]]][RemovedCustomer[j]] + d[RemovedCustomer[j]][Route[SelectedCustomer2[j] + 1][SelectedRoute2[j]]] - d[Route[SelectedCustomer2[j]][SelectedRoute2[j]]][Route[SelectedCustomer2[j] + 1][SelectedRoute2[j]]]));
				TempCost2 = Cost - RCost[1] * yPtr[Counter[SelectedRoute2[j]]][SelectedRoute2[j]];
				CostDifference[j] = abs(TempCost1 - TempCost2);
			}
			if (AddPoint == 1) {
				Regret = 0;
				if (RegretPossibility[j] == 1) {
					Max = 0;
					if (CostDifference[j] >= Max) {
						Max = CostDifference[j];
						SelectedNode = j;
						SelectedCustomer = SelectedCustomer1[j];
						SelectedRoute = SelectedRoute1[j];
						Regret = 1;
					}
				}
				if (Regret == 0) {
					if (SelectedRoute1[j] > 0) {
						SelectedNode = j;
						SelectedCustomer = SelectedCustomer1[j];
						SelectedRoute = SelectedRoute1[j];
					}
				}
				for (int kk = 1; kk <= NoVehicles; kk++) { //InitialNoRoutes
					if (Counter[kk] == 1) {
						if (kk == SelectedRoute) {
							NewRoute = 1;
						}
						else {
							Route[Counter[kk]][kk] = 0;
							Counter[kk] = 0;
						}
					}
				}

				if (NewRoute == 1) {
					NoRoutes++;
					if (MaxEnergyCapacity[SelectedRoute] < EnergyLimit) {
						Cost += (RCost[1] * Ratio * MaxEnergyCapacity[SelectedRoute]);
						Cost += AcquisitionCost[SelectedRoute];
						NoEV++;
					}
					else {
						Cost += AcquisitionCost[SelectedRoute];
						NoCV++;
					}
				}
				CustomerInsertion(SelectedRoute, SelectedCustomer, SelectedNode);

				/*	int Infeasibility = 0;
					for ( k = 1; k <= NoRoutes; k++){
						for ( i = 1; i <= Counter[k]; i++ ){
							if ( yPtr[i][k] < 0){
								Infeasibility = 1;
								break;
							}
						}
					}
				if ( Infeasibility == 1){
					GreedyStationInsertion1();

					}*/
				if (NewRoute == 1) {
					SortRoutes(SelectedRoute);
				}
			}
			j = 0;
		}
	}
}

void NoiseCustomerInsertion3() {
	int i, j, k, l, SelectedRoute, SelectedCustomer, SelectedNode, CounterSpecial, NewRoute, AddPoint;
	float Distance, Min, NoiseParameter;
	NoiseParameter = 0.1;
	int* Check;

	Check = new int[NoCustomers + 1];
	for (i = 0; i <= NoCustomers; i++) {
		Check[i] = 0;
	}

	for (j = 1; j <= CounterRemoval; j++) {
		if (Check[RemovedCustomer[j]] == 0) {
			NewRoute = 0;
			AddPoint = 0;
			CounterSpecial = 0;
			Min = BigNumber;
			Check[RemovedCustomer[j]] = 1;
			for (k = 1; k <= NoVehicles; k++) { // InitialNoRoutes
				if (Counter[k] == 0) {
					Counter[k] = 1;
					Route[Counter[k]][k] = NoVertices;

				}
				for (i = 0; i <= Counter[k] - 1; i++) {
					if (Counter[k] == 1) {
						TempQ[Counter[k]][k] = qPtr[0][k];
					}
					else {
						TempQ[Counter[k]][k] = qPtr[Counter[k]][k] - Demand[RemovedCustomer[j]];  //  Update Load for the end depot
					}
					Distance = d[Route[i][k]][RemovedCustomer[j]] + d[RemovedCustomer[j]][Route[i + 1][k]] - d[Route[i][k]][Route[i + 1][k]];
					double random_number = distribution(generator);
					Distance = Distance + MaxDistance * NoiseParameter * (float) random_number;
					if (Distance < Min && TempQ[Counter[k]][k] >= 0) { /// Check for load capacity
					// Check for time window
						CheckTimeWindowFunction(i, j, k);
						if (TimeWindowCheck == 0) {
							AddPoint = 1;
							Min = Distance;
							SelectedRoute = k;
							SelectedCustomer = i;
							SelectedNode = j;
						}
					}
				}
			}
			if (AddPoint == 1) {
				for (int kk = 1; kk <= NoVehicles; kk++) { //InitialNoRoutes
					if (Counter[kk] == 1) {
						if (kk == SelectedRoute) {
							NewRoute = 1;
						}
						else {
							Route[Counter[kk]][kk] = 0;
							Counter[kk] = 0;
						}
					}
				}

				if (NewRoute == 1) {
					NoRoutes++;
					if (MaxEnergyCapacity[SelectedRoute] < EnergyLimit) {
						Cost += (RCost[1] * Ratio * MaxEnergyCapacity[SelectedRoute]);
						Cost += AcquisitionCost[SelectedRoute];
						NoEV++;
					}
					else {
						Cost += AcquisitionCost[SelectedRoute];
						NoCV++;
					}
				}
				CustomerInsertion(SelectedRoute, SelectedCustomer, SelectedNode);
				/*	int Infeasibility = 0;
					for ( k = 1; k <= NoRoutes; k++){
						for ( i = 1; i <= Counter[k]; i++ ){
							if ( yPtr[i][k] < 0){
								Infeasibility = 1;
								break;
							}
						}
					}
				if ( Infeasibility == 1){
					GreedyStationInsertion1();

					}*/
				if (NewRoute == 1) {
					SortRoutes(SelectedRoute);
				}
			}
			j = 0;
		}
	}
}

void InitialSolutionI(int index1, int index2, int index3, int index4) {
	int i, j, k, l, m, SelectedRoute, Feasibility;

	sprintf_s(OutName, "%s%d-%d-%d-%d", "C:\\PhD\\Paper 2\\Results\\Result", index1, index2, index3, index4);  /////// ?????????
	strcat_s(OutName, ".txt");
	errno_t err = fopen_s(&Out, OutName, "w");
	NoRoutes = 0;
	Cost = 0;
	for (j = 1; j <= NoCustomers; j++) {
		SortI[j] = j;
	}
	for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
		SortII[i] = i;
	}
	for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
		if (Rate[i] < 0.2) {
			Technology[i] = 3;
		}
		else {
			Technology[i] = 2;
		}
	}
	/*for(m = NoCustomers + 1; m <= NoCustomers + NoStations; m++){
			Penalty[m] = Rate[m];
			SortIII[m] = m;
	}
	for(m = 1; m <= NoChargers; m++){
		SortIII[m] = m;
	}

	//// Sort Chargers based on their penalty: Cost * Time or Rate ///////
	for( i = 1; i <= NoChargers - 1; i++){
		for( j = i + 1; j <= NoChargers; j++){
			if(Penalty[SortIII[i]] > Penalty[SortIII[j]]){
				int temp;
				temp = SortIII[j];
				for(l = j-1 ; l >= i; l--){
					SortIII[l+1] = SortIII[l];
					//cout<<SortIII[k+1]<<endl;
				}
				SortIII[i] = temp;
				//cout<<SortIII[j]<<endl;
			}
		}
	}
	*/
	//// Sort Customers based on their latest start time ////
	for (i = 1; i <= NoCustomers - 1; i++) {
		for (j = i + 1; j <= NoCustomers; j++) {
			if (lTime[SortI[i]] > lTime[SortI[j]]) {
				//cout<<lTime[i]<<endl<<lTime[j]<<endl;
				int temp;
				temp = SortI[j];
				for (l = j - 1; l >= i; l--) {
					SortI[l + 1] = SortI[l];
					//cout<<Sort[k+1]<<endl;
				}
				SortI[i] = temp;
				//cout<<SortI[j]<<endl;
			}
		}
	}

	m = 1;
	for (k = 1; k <= NoVehicles; k++) {
		Feasibility = 0;
		for (i = m; i <= NoCustomers; i++) {
			if (Demand[SortI[i]] <= MaxCapacity[k]) {      //Load Check
				if (t[0][SortI[i]] <= lTime[SortI[i]]) {  //Time Window Check
					if (d[0][SortI[i]] * EnergyConsumption[k] <= Ratio * MaxEnergyCapacity[k]) { //Energy Check
						Feasibility = 1;
						m++;
						break;
					}
					else {
						for (j = NoCustomers; j <= NoCustomers + NoStations; j++) {
							if (d[0][j] <= Ratio * MaxEnergyCapacity[k] && d[j][SortI[i]] <= Ratio * MaxEnergyCapacity[k]) {
								Feasibility = 1;
								m++;
								break;
							}
						}
					}
				}
			}
		}
		if (Feasibility == 0) {
			fprintf(Out, "%s%s", " There is no feasible solution for this problem ", "\n");
			exit(EXIT_FAILURE);
		}
	}

	RoutedCustomers = 0;
	for (k = 1; k <= NoVehicles; k++) {
		if (RoutedCustomers < NoCustomers) {
			Counter[k] = 0;
			Route[0][k] = 0;
			uPtr[0][k] = 0;
			yPtr[0][k] = Ratio * MaxEnergyCapacity[k];
			qPtr[0][k] = MaxCapacity[k];
			for (m = 1; m <= NoCustomers; m++) {
				if (Seed[SortI[m]] == 0) {
					if (Demand[SortI[m]] <= qPtr[0][k]) {   // Check Load for the selected node
						TempU[SortI[m]] = uPtr[0][k] + t[Route[0][k]][SortI[m]];
						UpdateTimeI(SortI[m], NoVertices);
						if (TempU[SortI[m]] <= lTime[SortI[m]] && TempU[NoVertices] <= lTime[NoVertices]) { //Check Time Window for the selected node and the last node
							TempY[SortI[m]][k] = yPtr[0][k] - EnergyConsumption[k] * d[Route[0][k]][SortI[m]];
							if (TempY[SortI[m]][k] > 0) { //Check Energy for the selected node  //Yes
								Route[1][k] = SortI[m];
								RouteLength[k] = d[Route[0][k]][Route[1][k]];
								Seed[SortI[m]] = 1;
								Counter[k] ++;
								RoutedCustomers++;
								uPtr[Counter[k]][k] = TempU[SortI[m]];
								yPtr[Counter[k]][k] = yPtr[Counter[k] - 1][k] - EnergyConsumption[k] * d[Route[Counter[k] - 1][k]][Route[Counter[k]][k]];
								qPtr[Counter[k]][k] = qPtr[Counter[k] - 1][k] - Demand[Route[Counter[k] - 1][k]];
								TempY[NoVertices][k] = yPtr[Counter[k]][k] - EnergyConsumption[k] * d[Route[Counter[k]][k]][NoVertices];
								if (TempY[NoVertices][k] > 0) { // Check Energy for the last node/end depot //Yes
									RouteLength[k] += d[Route[1][k]][Route[2][k]];
									Counter[k]++;
									Route[Counter[k]][k] = NoVertices;
									uPtr[Counter[k]][k] = TempU[NoVertices];
									qPtr[Counter[k]][k] = qPtr[Counter[k] - 1][k] - Demand[Route[Counter[k] - 1][k]];
									yPtr[Counter[k]][k] = TempY[NoVertices][k];
									m = NoCustomers + 1;

								}
								else {
									///// Sort Stations based on their distance from the customer ///////
									SortStations(Route[Counter[k]][k], NoVertices);
									if (yPtr[Counter[k]][k] >= EnergyConsumption[k] * d[Route[Counter[k]][k]][SortII[NoCustomers + 1]]) { //Check if the vehicle has enough energy to reach the station
										TechnologySelectionI(SortII[NoCustomers + 1], k, NoVertices);
										if (TempU[NoVertices] <= lTime[NoVertices]) {  // Check Time Window for the end depot //Yes
											Counter[k] ++;
											Route[Counter[k]][k] = SortII[NoCustomers + 1];
											Route[Counter[k] + 1][k] = NoVertices;
											for (i = Counter[k]; i <= Counter[k] + 1; i++) {
												yPtr[i][k] = TempY[Route[i][k]][k];
												uPtr[i][k] = TempU[Route[i][k]];
												qPtr[i][k] = qPtr[i - 1][k] - Demand[Route[i - 1][k]];
												RouteLength[k] += d[Route[i - 1][k]][Route[i][k]];
											}
											vPtr[Counter[k]][Technology[SortII[NoCustomers + 1]]][k] = TempV[SortII[NoCustomers + 1]][Technology[SortII[NoCustomers + 1]]][k];
											Counter[k] ++;
											CounterStation[SortII[NoCustomers + 1]] ++;
											m = NoCustomers + 1;
										}
									}
									else {
										RouteLength[k] = 0;
										Seed[SortI[m]] = 0;
										RoutedCustomers--;
										Counter[k] = 0;
										Route[0][k] = 0;
										uPtr[0][k] = 0;
										yPtr[0][k] = Ratio * MaxEnergyCapacity[k];
										qPtr[0][k] = MaxCapacity[k];
										goto label1;  //First visit a station and selected node
									}
								}
							}
							else {
							label1:
								Counter[k]++;
								Route[Counter[k]][k] = NoVertices;
								yPtr[Counter[k]][k] = Ratio * MaxEnergyCapacity[k];
								///// Sort Stations based on their distance from the customer ///////
								SortStations(Route[Counter[k]][k], SortI[m]);
								// Update Time and energy for the station and selected node
								TechnologySelectionI(SortII[NoCustomers + 1], k, SortI[m]);
								if (TempU[SortI[m]] <= lTime[SortI[m]]) { //Check Time window for the selected node  ??
									Route[Counter[k]][k] = SortII[NoCustomers + 1];
									Route[Counter[k] + 1][k] = SortI[m];
									Route[Counter[k] + 2][k] = SortII[NoCustomers + 1];
									Route[Counter[k] + 3][k] = NoVertices;
									for (i = Counter[k]; i <= Counter[k] + 1; i++) {
										yPtr[i][k] = TempY[Route[i][k]][k];
										uPtr[i][k] = TempU[Route[i][k]];
										qPtr[i][k] = qPtr[i - 1][k] - Demand[Route[i - 1][k]];
										RouteLength[k] += d[Route[i - 1][k]][Route[i][k]];
									}
									vPtr[Counter[k]][Technology[SortII[NoCustomers + 1]]][k] = TempV[SortII[NoCustomers + 1]][Technology[SortII[NoCustomers + 1]]][k];
									CounterStation[SortII[NoCustomers + 1]] ++;
									Seed[SortI[m]] = 1;
									UpdateTimeI(Route[Counter[k] - 1][k], Route[Counter[k]][k]);
									for (i = Counter[k] + 2; i <= Counter[k] + 3; i++) {
										if (Route[i][k] > NoCustomers&& Route[i][k] < NoVertices) {
											uPtr[i][k] = uPtr[i - 1][k] + ServiceTime[Route[i - 1][k]] + t[Route[i - 1][k]][Route[i][k]];
											yPtr[i][k] = yPtr[i - 1][k] - EnergyConsumption[k] * d[Route[i - 1][k]][Route[i][k]];
											qPtr[i][k] = qPtr[i - 1][k] - Demand[Route[i - 1][k]];
											RouteLength[k] += d[Route[i - 1][k]][Route[i][k]];
											CounterStation[Route[i - 1][k]] ++;
										}
										else {
											vPtr[i - 1][Technology[Route[i - 1][k]]][k] = 0.8 * MaxEnergyCapacity[k] - yPtr[i - 1][k];
											yPtr[i][k] = yPtr[i - 1][k] + vPtr[i - 1][Technology[Route[i - 1][k]]][k] - EnergyConsumption[k] * d[Route[i - 1][k]][Route[i][k]];
											uPtr[i][k] = uPtr[i - 1][k] + Rate[Route[i - 1][k]] * vPtr[i - 1][Technology[Route[i - 1][k]]][k] + ServiceTime[Route[i - 1][k]] + t[Route[i - 1][k]][Route[i][k]];
											qPtr[i][k] = qPtr[i - 1][k] - Demand[Route[i - 1][k]];
											RouteLength[k] += d[Route[i - 1][k]][Route[i][k]];
										}
									}
									Counter[k] += 3;
									if (uPtr[Counter[k]][k] <= lTime[Counter[k]]) { //Check Time window for the end depot ??
										//Update Load
										RoutedCustomers++;
										m = NoCustomers + 1;
									}
									else {
										for (i = 0; i <= Counter[k]; i++) {
											if (Route[i][k] > NoCustomers&& Route[i][k] < NoVertices) {
												vPtr[i][Technology[Route[i][k]]][k] = 0;
											}
											Route[i][k] = 0;
											yPtr[i][k] = 0;
											qPtr[i][k] = 0;
											uPtr[i][k] = 0;
										}
										CounterStation[SortII[NoCustomers + 1]] = -2;
										Seed[SortI[m]] = 0;
										RouteLength[k] = 0;
										Counter[k] = 0;
										yPtr[0][k] = Ratio * MaxEnergyCapacity[k];
										qPtr[0][k] = MaxCapacity[k];

									}
								}
								else {
									Route[Counter[k]][k] = 0;
									yPtr[Counter[k]][k] = 0;
									Counter[k] = 0;
									Route[0][k] = 0;
									uPtr[0][k] = 0;
									yPtr[0][k] = Ratio * MaxEnergyCapacity[k];
									qPtr[0][k] = MaxCapacity[k];
									CounterStation[SortII[NoCustomers + 1]] = -2;
									Seed[SortI[m]] = 0;
								}
							}
							if (Counter[k] >= 2) {
								RouteConstruction(k);
							}
						}
					}
				}
			}
		}
	}
	if (RoutedCustomers < NoCustomers) {
		fprintf(Out, "%s%s", " There is no feasible solution for this problem ", "\n");
		exit(EXIT_FAILURE);
	}
	for (k = 1; k <= NoVehicles; k++) {
		if (Route[1][k] != 0) {
			NoRoutes++;
			SelectedRoute = k;
			SortRoutes(SelectedRoute);
		}
	}
	for (k = 1; k <= NoRoutes; k++) {
		fprintf(Out, "%s%d%s", "Route ", k, ": ");
		for (i = 0; i <= Counter[k]; i++) {
			fprintf(Out, "%d%s", Route[i][k], " ");
		}
		fprintf(Out, "%s", "\n");
	}
	fprintf(Out, "%s", "\n");
	for (k = 1; k <= NoRoutes; k++) {
		for (i = 0; i <= Counter[k] - 1; i++) {
			Cost += (TCost[k] * d[Route[i][k]][Route[i + 1][k]]);
				if (MaxEnergyCapacity[k] > EnergyLimit) {
					Emissions += (Lambda * CurbWeight * Rho * Sigma2nd * (d[Route[i][k]][Route[i + 1][k]])); 
					Emissions += (Lambda * EngineFriction * EngineSpeed * EngineDisplacement * (d[Route[i][k]][Route[i + 1][k]]/speed));
					Emissions += (Lambda * Epsilon2nd * Rho * d[Route[i][k]][Route[i + 1][k]] * speed * speed);
					Emissions += (Rho * Sigma2nd * Lambda * (d[Route[i][k]][Route[i + 1][k]]) * qPtr[i][k]);
				}
			//Cost += d[Route[i][k]][Route[i+1][k]];
		}
	}
	for (k = 1; k <= NoRoutes; k++) {
		if (Route[1][k] != 0) {
			if (MaxEnergyCapacity[k] < EnergyLimit) {
				Cost += (RCost[1] * Ratio * MaxEnergyCapacity[k]);
				Cost += AcquisitionCost[k];
				NoEV++;
			}
			else {
				Cost += AcquisitionCost[k];
				NoCV++;
			}
		}
	}
	for (k = 1; k <= NoRoutes; k++) {
		for (i = 0; i <= Counter[k]; i++) {
			fprintf(Out, "%s%d%s%d%s%f%s", "y[", i, "][", k, "] = ", yPtr[i][k], "\n");
		}
		if (MaxEnergyCapacity[k] < EnergyLimit) {
			Cost -= (RCost[1] * yPtr[Counter[k]][k]);
		}

		fprintf(Out, "%s", "\n");
	}
	for (k = 1; k <= NoRoutes; k++) {
		for (i = 0; i <= Counter[k]; i++) {
			fprintf(Out, "%s%d%s%d%s%f%s", "q[", i, "][", k, "] = ", qPtr[i][k], "\n");
		}
		fprintf(Out, "%s", "\n");
	}
	for (k = 1; k <= NoRoutes; k++) {
		for (i = 0; i <= Counter[k]; i++) {
			fprintf(Out, "%s%d%s%d%s%f%s", "u[", i, "][", k, "] = ", uPtr[i][k], "\n");
		}
	}
	fprintf(Out, "%s", "\n");
	for (k = 1; k <= NoRoutes; k++) {
		for (i = 0; i <= Counter[k]; i++) {
			for (m = 1; m <= NoChargers; m++) {
				fprintf(Out, "%s%d%s%d%s%d%s%f%s", "v[", i, "][", m, "][", k, "] = ", vPtr[i][m][k], "\n");
				if (yPtr[i][k] < EnergyLimit) {
					Cost += (RCost[m] * vPtr[i][m][k]);
				}
			}
			fprintf(Out, "%s", "\n");
		}
		fprintf(Out, "%s", "\n");
	}
	fprintf(Out, "%s%f%s", "TotalCost= ", Cost, "\n");
}

void UpdateCurrent() {
	int i, j, k, m;

	for (k = 1; k <= CurrentNoRoutes; k++) {
		for (i = 0; i <= CurrentCounter[k]; i++) {
			CurrentRoute[i][k] = 0;
			CurrentU[i][k] = 0;
			CurrentQ[i][k] = 0;
			CurrentY[i][k] = 0;
		}
		CurrentRouteLength[k] = 0;
		CurrentMaxEnergyCapacity[k] = 0;
		CurrentTCost[k] = 0;
		CurrentAcquisitionCost[k] = 0;
	}
	for (k = 0; k <= CurrentNoRoutes; k++) {
		for (m = 0; m <= NoChargers; m++) {
			for (i = 0; i <= CurrentCounter[k]; i++) {
				CurrentV[i][m][k] = 0;
			}
		}
		CurrentCounter[k] = 0;
	}
	CurrentNoEV = NoEV;
	CurrentNoCV = NoCV;
	CurrentCost = Cost;
	CurrentWeightedSum = WeightedSum;
	CurrentEmissions = Emissions;
	CurrentNoRoutes = NoRoutes;
	for (k = 1; k <= NoRoutes; k++) {
		CurrentMaxEnergyCapacity[k] = MaxEnergyCapacity[k];
		CurrentTCost[k] = TCost[k];
		CurrentAcquisitionCost[k] = AcquisitionCost[k];
		CurrentRouteLength[k] = RouteLength[k];
		CurrentCounter[k] = Counter[k];
		for (i = 0; i <= Counter[k]; i++) {
			CurrentRoute[i][k] = Route[i][k];
			CurrentU[i][k] = uPtr[i][k];
			CurrentQ[i][k] = qPtr[i][k];
			CurrentY[i][k] = yPtr[i][k];
		}
	}
	for (k = 0; k <= NoRoutes; k++) {
		for (m = 0; m <= NoChargers; m++) {
			for (i = 0; i <= Counter[k]; i++) {
				CurrentV[i][m][k] = vPtr[i][m][k];
			}
		}
	}
	for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
		CurrentCounterStation[i] = CounterStation[i];
	}
}

void UpdateBest() {
	int i, j, k, m;

	for (k = 1; k <= CurrentNoRoutes; k++) {
		for (i = 0; i <= CurrentCounter[k]; i++) {
			CurrentRoute[i][k] = 0;
			CurrentU[i][k] = 0;
			CurrentQ[i][k] = 0;
			CurrentY[i][k] = 0;
		}
		CurrentRouteLength[k] = 0;
		CurrentMaxEnergyCapacity[k] = 0;
		CurrentTCost[k] = 0;
		CurrentAcquisitionCost[k] = 0;
	}
	for (k = 0; k <= CurrentNoRoutes; k++) {
		for (m = 0; m <= NoChargers; m++) {
			for (i = 0; i <= CurrentCounter[k]; i++) {
				CurrentV[i][m][k] = 0;
			}
		}
		CurrentCounter[k] = 0;
	}
	for (k = 1; k <= BestNoRoutes; k++) {
		for (i = 0; i <= BestCounter[k]; i++) {
			BestRoute[i][k] = 0;
			BestU[i][k] = 0;
			BestQ[i][k] = 0;
			BestY[i][k] = 0;
		}
		BestRouteLength[k] = 0;
	}
	for (k = 0; k <= BestNoRoutes; k++) {
		for (m = 0; m <= NoChargers; m++) {
			for (i = 0; i <= BestCounter[k]; i++) {
				BestV[i][m][k] = 0;
			}
		}
		BestCounter[k] = 0;
	}
	BestNoEV = NoEV;
	CurrentNoEV = NoEV;
	BestNoCV = NoCV;
	CurrentNoCV = NoCV;
	BestCost = Cost;
	CurrentCost = Cost;
	BestWeightedSum = WeightedSum;
	CurrentWeightedSum = WeightedSum;
	CurrentEmissions = Emissions;
	BestEmissions = Emissions;
	BestNoRoutes = NoRoutes;
	CurrentNoRoutes = NoRoutes;
	for (k = 1; k <= NoVehicles; k++) {
		BestEnergyConsumption[k] = EnergyConsumption[k];
		CurrentEnergyConsumption[k] = EnergyConsumption[k];
		BestMaxEnergyCapacity[k] = MaxEnergyCapacity[k];
		CurrentMaxEnergyCapacity[k] = MaxEnergyCapacity[k];
		BestTCost[k] = TCost[k];
		CurrentTCost[k] = TCost[k];
		BestAcquisitionCost[k] = AcquisitionCost[k];
		CurrentAcquisitionCost[k] = AcquisitionCost[k];
	}
	for (k = 1; k <= NoRoutes; k++) {
		BestRouteLength[k] = RouteLength[k];
		CurrentRouteLength[k] = RouteLength[k];
		BestCounter[k] = Counter[k];
		CurrentCounter[k] = Counter[k];
		for (i = 0; i <= Counter[k]; i++) {
			BestRoute[i][k] = Route[i][k];
			CurrentRoute[i][k] = Route[i][k];
			BestU[i][k] = uPtr[i][k];
			CurrentU[i][k] = uPtr[i][k];
			BestQ[i][k] = qPtr[i][k];
			CurrentQ[i][k] = qPtr[i][k];
			BestY[i][k] = yPtr[i][k];
			CurrentY[i][k] = yPtr[i][k];
		}
	}
	for (k = 0; k <= NoRoutes; k++) {
		for (m = 0; m <= NoChargers; m++) {
			for (i = 0; i <= Counter[k]; i++) {
				BestV[i][m][k] = vPtr[i][m][k];
				CurrentV[i][m][k] = vPtr[i][m][k];
			}
		}
	}
	for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
		BestCounterStation[i] = CounterStation[i];
		CurrentCounterStation[i] = CounterStation[i];
	}
}

void UpdateWeights() {
	int i, s;
	float Sum;
	s = 0;

	// Update weights of CR and CI algorithms
	for (i = NSR + NSI + 1; i <= NSR + NSI + NCR + NCI; i++) {
		if (Score[i] > 0) {
			Weight[s + 1][i] = Weight[s][i] * (1 - RouletteWheelPar) + RouletteWheelPar * (float) Score[i] / (float) NoUse[i];
		}
	}
	// Define weights for Customer Removal Operators
	Sum = 0;
	for (i = NSR + NSI + 1; i <= NSR + NSI + NCR; i++) {
		Sum += Weight[s + 1][i];
	}
	for (i = NSR + NSI + 1; i <= NSR + NSI + NCR; i++) {
		Probability[s + 1][i] = Weight[s + 1][i] / Sum;
	}
	for (i = NSR + NSI + 1; i <= NSR + NSI + NCR; i++) {
		Score[i] = 0;
	}
	Pr[NSR + NSI + 1] = Probability[s + 1][NSR + NSI + 1];
	for (i = NSR + NSI + 2; i <= NSR + NSI + NCR; i++) {
		Pr[i] = Pr[i - 1] + Probability[s + 1][i];
	}
	// Define weights for Customer Insertion Operators
	Sum = 0;
	for (i = NSR + NSI + NCR + 1; i <= NSR + NSI + NCR + NCI; i++) {
		Sum += Weight[s + 1][i];
	}
	for (i = NSR + NSI + NCR + 1; i <= NSR + NSI + NCR + NCI; i++) {
		Probability[s + 1][i] = Weight[s + 1][i] / Sum;
	}
	for (i = NSR + NSI + NCR + 1; i <= NSR + NSI + NCR + NCI; i++) {
		Score[i] = 0;
	}
	Pr[NSR + NSI + NCR + 1] = Probability[s + 1][NSR + NSI + NCR + 1];
	for (i = NSR + NSI + NCR + 2; i <= NSR + NSI + NCR + NCI; i++) {
		Pr[i] = Pr[i - 1] + Probability[s + 1][i];
	}
	// Update weights of SR and SI algorithms
	for (i = 1; i <= NSR + NSI; i++) {
		if (Score[i] > 0) {
			Weight[s + 1][i] = Weight[s][i] * (1 - RouletteWheelPar) + RouletteWheelPar * (float) Score[i] / (float) NoUse[i];
		}
	}
	// Define weights for Station Removal Operators
	Sum = 0;
	for (i = 1; i <= NSR; i++) {
		Sum += Weight[s + 1][i];
	}
	for (i = 1; i <= NSR; i++) {
		Probability[s + 1][i] = Weight[s + 1][i] / Sum;
	}
	for (i = 1; i <= NSR; i++) {
		Score[i] = 0;
	}
	Pr[1] = Probability[s + 1][1];
	for (i = 2; i <= NSR; i++) {
		Pr[i] = Pr[i - 1] + Probability[s + 1][i];
	}
	// Define weights for Station Insertion Operators
	Sum = 0;
	for (i = NSR + 1; i <= NSR + NSI; i++) {
		Sum += Weight[s + 1][i];
	}
	for (i = NSR + 1; i <= NSR + NSI; i++) {
		Probability[s + 1][i] = Weight[s + 1][i] / Sum;
	}
	for (i = NSR + 1; i <= NSR + NSI; i++) {
		Score[i] = 0;
	}
	Pr[NSR + 1] = Probability[s + 1][NSR + 1];
	for (i = NSR + 2; i <= NSR + NSI; i++) {
		Pr[i] = Pr[i - 1] + Probability[s + 1][i];
	}
}

void Swap() {
	int i, j, k, l, SelectedNode1, SelectedNode2;
	float Min = BigNumber;

	// It should be better than the original cost or not ?
	for (k = 1; k <= NoRoutes - 1; k++) {
		for (l = k + 1; l <= NoRoutes; l++) {
			for (i = 1; i <= Counter[k] - 1; i++) {
				for (j = 1; j <= Counter[l]; j++) {
					if (Route[i][k] <= NoCustomers) {
						if (Route[j][l] <= NoCustomers) {
							SelectedNode1 = Route[i][k];
							SelectedNode2 = Route[j][l];
							RemoveCustomer(k, i);
							RemoveCustomer(l, j);
							CustomerInsertion(l, j - 1, CounterRemoval - 1);
							if (Route[j][l] == SelectedNode1) {
								CustomerInsertion(k, i - 1, CounterRemoval);
								if (Route[i][k] == SelectedNode2) {
									if (Cost < Min) {
										Min = Cost;
										continue;
									}
									else {
										RemoveCustomer(k, i);
										RemoveCustomer(l, j);
										CustomerInsertion(l, j - 1, CounterRemoval - 1);
										CustomerInsertion(k, i - 1, CounterRemoval);

									}
								}
								else {
									CustomerInsertion(k, i - 1, CounterRemoval);
								}
							}
							else {
								CustomerInsertion(l, j - 1, CounterRemoval - 1);
							}
						}
					}
				}
			}
		}
	}
}

void ALNS() {
	float r, PercentRemovalC, CoolingPar, ProbAcceptance, Temprature, Sum, Value1, Value2, Value3;
	int i, j, m, k, s, OuterIter, InnerIter, RouteIter, Sigma1, Sigma2, Sigma3, Beta, OperatorCounter,
		Infeasibility, Random, NoLevel2, NoLevel3, ValueS, ValueC, ValueR;
	float SumValues = 0;
	float Epsilon = 0;
	float EConstraint = 0;
	float Weight2nd = 0.1;
	int BiIter = 20;
	NoSolution = 0;

	Temprature = 40;
	RouletteWheelPar = 0.25;
	CoolingPar = 0.9994;
	OuterIter = 220;
	InnerIter = 250; //2500
	RouteIter = 20; //1250 Maybe make it less is better idea
	ProbAcceptance = 0;
	IterO = 0;
	Sigma1 = 15;
	Sigma2 = 10; //10
	Sigma3 = 5; //2 5
	Beta = 10;  //10 3
	Infeasibility = 0;
	s = 0;
	NoLevel2 = 0;
	NoLevel3 = 0;
	//Random=No;


	// Start with intital solution
	BestCost = Cost;
	CurrentCost = Cost;
	InitialCost = Cost;
	BestEmissions = Emissions;
	CurrentEmissions = Emissions;
	InitialEmissions = Emissions;
	BestNoRoutes = NoRoutes;
	CurrentNoRoutes = NoRoutes;
	InitialNoRoutes = NoRoutes;
	BestNoEV = NoEV;
	CurrentNoEV = NoEV;
	InitialNoEV = NoEV;
	BestNoCV = NoCV;
	CurrentNoCV = NoCV;
	InitialNoCV = NoCV;
	for (k = 1; k <= NoVehicles; k++) {
		BestEnergyConsumption[k] = EnergyConsumption[k];
		CurrentEnergyConsumption[k] = EnergyConsumption[k];
		InitialEnergyConsumption[k] = EnergyConsumption[k];
		BestMaxEnergyCapacity[k] = MaxEnergyCapacity[k];
		CurrentMaxEnergyCapacity[k] = MaxEnergyCapacity[k];
		InitialMaxEnergyCapacity[k] = MaxEnergyCapacity[k];
		BestTCost[k] = TCost[k];
		CurrentTCost[k] = TCost[k];
		InitialTCost[k] = TCost[k];
		BestAcquisitionCost[k] = AcquisitionCost[k];
		CurrentAcquisitionCost[k] = AcquisitionCost[k];
		InitialAcquisitionCost[k] = AcquisitionCost[k];
	}
	for (k = 1; k <= NoRoutes; k++) {
		BestRouteLength[k] = RouteLength[k];
		CurrentRouteLength[k] = RouteLength[k];
		InitialRouteLength[k] = RouteLength[k];
		BestCounter[k] = Counter[k];
		CurrentCounter[k] = Counter[k];
		InitialCounter[k] = Counter[k];

		for (i = 0; i <= Counter[k]; i++) {
			BestRoute[i][k] = Route[i][k];
			CurrentRoute[i][k] = Route[i][k];
			InitialRoute[i][k] = Route[i][k];
			BestU[i][k] = uPtr[i][k];
			CurrentU[i][k] = uPtr[i][k];
			InitialU[i][k] = uPtr[i][k];
			BestQ[i][k] = qPtr[i][k];
			CurrentQ[i][k] = qPtr[i][k];
			InitialQ[i][k] = qPtr[i][k];
			BestY[i][k] = yPtr[i][k];
			CurrentY[i][k] = yPtr[i][k];
			InitialY[i][k] = yPtr[i][k];
		}
	}
	for (k = 1; k <= NoVehicles; k++) {
		BestEnergyConsumption[k] = EnergyConsumption[k];
		CurrentEnergyConsumption[k] = EnergyConsumption[k];
		InitialEnergyConsumption[k] = EnergyConsumption[k];
		BestMaxEnergyCapacity[k] = MaxEnergyCapacity[k];
		CurrentMaxEnergyCapacity[k] = MaxEnergyCapacity[k];
		InitialMaxEnergyCapacity[k] = MaxEnergyCapacity[k];
		BestTCost[k] = TCost[k];
		CurrentTCost[k] = TCost[k];
		InitialTCost[k] = TCost[k];
		BestAcquisitionCost[k] = AcquisitionCost[k];
		CurrentAcquisitionCost[k] = AcquisitionCost[k];
		InitialAcquisitionCost[k] = AcquisitionCost[k];
	}
	for (k = 0; k <= NoRoutes; k++) {
		for (m = 0; m <= NoChargers; m++) {
			for (i = 0; i <= Counter[k]; i++) {
				BestV[i][m][k] = vPtr[i][m][k];
				CurrentV[i][m][k] = vPtr[i][m][k];
				InitialV[i][m][k] = vPtr[i][m][k];
			}
		}
	}
	for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
		BestCounterStation[i] = CounterStation[i];
		CurrentCounterStation[i] = CounterStation[i];
		InitialCounterStation[i] = CounterStation[i];
	}

	// Define weights for the first iteration for Station Removal Operators
	Sum = 0;
	for (i = 1; i <= NSR; i++) {
		Sum += Weight[s][i];
	}
	for (i = 1; i <= NSR; i++) {
		Probability[s][i] = Weight[s][i] / Sum;
	}
	for (i = 1; i <= NSR; i++) {
		Score[i] = 0;
	}
	Pr[1] = Probability[s][1];
	for (i = 2; i <= NSR; i++) {
		Pr[i] = Pr[i - 1] + Probability[s][i];
	}
	// Define weights for the first iteration for Station Insertion Operators
	Sum = 0;
	for (i = NSR + 1; i <= NSR + NSI; i++) {
		Sum += Weight[s][i];
	}
	for (i = NSR + 1; i <= NSR + NSI; i++) {
		Probability[s][i] = Weight[s][i] / Sum;
	}
	for (i = NSR + 1; i <= NSR + NSI; i++) {
		Score[i] = 0;
	}
	Pr[NSR + 1] = Probability[s][NSR + 1];
	for (i = NSR + 2; i <= NSR + NSI; i++) {
		Pr[i] = Pr[i - 1] + Probability[s][i];
	}
	// Define weights for the first iteration for Customer Removal Operators
	Sum = 0;
	for (i = NSR + NSI + 1; i <= NSR + NSI + NCR; i++) {
		Sum += Weight[s][i];
	}
	for (i = NSR + NSI + 1; i <= NSR + NSI + NCR; i++) {
		Probability[s][i] = Weight[s][i] / Sum;
	}
	for (i = NSR + NSI + 1; i <= NSR + NSI + NCR; i++) {
		Score[i] = 0;
	}
	Pr[NSR + NSI + 1] = Probability[s][NSR + NSI + 1];
	for (i = NSR + NSI + 2; i <= NSR + NSI + NCR; i++) {
		Pr[i] = Pr[i - 1] + Probability[s][i];
	}
	// Define weights for the first iteration for Customer Insertion Operators
	Sum = 0;
	for (i = NSR + NSI + NCR + 1; i <= NSR + NSI + NCR + NCI; i++) {
		Sum += Weight[s][i];
	}
	for (i = NSR + NSI + NCR + 1; i <= NSR + NSI + NCR + NCI; i++) {
		Probability[s][i] = Weight[s][i] / Sum;
	}
	for (i = NSR + NSI + NCR + 1; i <= NSR + NSI + NCR + NCI; i++) {
		Score[i] = 0;
	}
	Pr[NSR + NSI + NCR + 1] = Probability[s][NSR + NSI + NCR + 1];
	for (i = NSR + NSI + NCR + 2; i <= NSR + NSI + NCR + NCI; i++) {
		Pr[i] = Pr[i - 1] + Probability[s][i];
	}
	Value1 = Value2 = Value3 = 1;

	while (IterO < OuterIter) {
		Iter = 0;
		IterO++;
		// Should I define a new function for this??
		Cost = BestCost;
		Emissions = BestEmissions;
		NoEV = BestNoEV;
		NoCV = BestNoCV;
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 0; i <= Counter[k]; i++) {
				Route[i][k] = 0;
				uPtr[i][k] = 0;
				qPtr[i][k] = 0;
				yPtr[i][k] = 0;
			}
			RouteLength[k] = 0;
		}
		for (k = 0; k <= NoRoutes; k++) {
			for (m = 0; m <= NoChargers; m++) {
				for (i = 0; i <= Counter[k]; i++) {
					vPtr[i][m][k] = 0;
				}
			}
			Counter[k] = 0;

		}
		NoRoutes = BestNoRoutes;
		for (k = 1; k <= NoVehicles; k++) {
			MaxEnergyCapacity[k] = BestMaxEnergyCapacity[k];
			TCost[k] = BestTCost[k];
			EnergyConsumption[k] = BestEnergyConsumption[k];
			AcquisitionCost[k] = BestAcquisitionCost[k];
		}
		for (k = 1; k <= BestNoRoutes; k++) {
			RouteLength[k] = BestRouteLength[k];
			Counter[k] = BestCounter[k];
			for (i = 0; i <= Counter[k]; i++) {
				Route[i][k] = BestRoute[i][k];
				uPtr[i][k] = BestU[i][k];
				qPtr[i][k] = BestQ[i][k];
				yPtr[i][k] = BestY[i][k];
			}
		}

		for (k = 0; k <= NoRoutes; k++) {
			for (m = 0; m <= NoChargers; m++) {
				for (i = 0; i <= Counter[k]; i++) {
					vPtr[i][m][k] = BestV[i][m][k];
				}
			}
		}
		for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
			CounterStation[i] = 0;
		}
		for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
			CounterStation[i] = BestCounterStation[i];
		}
		for (i = 0; i <= NoCustomers; i++) {
			RemovedCustomer[i] = 0;
		}
		CounterRemoval = 0;
		RoutedCustomers = NoCustomers;
		//Swap();
		Value1 = Value2 = Value3 = 1;
		while (Iter <= InnerIter) {
			OperatorCounter = 0;
			//Random+=114; //(123/114/172/114)
			//srand(Random);
			ValueS = ValueC = ValueR = 0;
			for (i = 1; i <= NSR + NSI + NCR + NCI; i++) {
				Score[i] = 0;
			}
			for (k = 1; k <= NoRoutes; k++) {
				if (uPtr[Counter[k]][k] > 540) {
					int violation = 1;
				}
				for (i = 1; i <= Counter[k]; i++) {
					if (uPtr[i][k] < 0) {
						int violation = 1;
					}
				}
			}
			SumValues = Value1 + Value2 + Value3;
			r = ((double)rand() / (RAND_MAX + 1));
			for (k = 1; k <= BestNoRoutes; k++) {
				for (i = 0; i <= BestCounter[k]; i++) {
					if (BestU[i][k] < 0) {
						int Violation = 1;
					}
				}
			}
			for (k = 1; k <= BestNoRoutes; k++) {
				if (BestU[BestCounter[k]][k] > 540) {
					int Violation = 1;
				}
			}
			if (r < (Value1 / SumValues)) {
				ValueS = 1;
				//Station Removal Operator Selection //
				r = ((double)rand() / (RAND_MAX + 1));
				if (r < Pr[1]) {
					RandomStationRemoval1();
					Score[1] = 1;
					NoUse[1]++;
				}
				else if (r > Pr[1] && r < Pr[2]) {
					WorstDistanceStationRemoval2();
					Score[2] = 1;
					NoUse[2]++;
				}
				else if (r > Pr[2] && r < Pr[3]) {
					LeastUsedStationRemoval3();
					Score[3] = 1;
					NoUse[3]++;
				}
				else if (r > Pr[3] && r < Pr[4]) {
					ExpensiveStationRemoval4();
					Score[4] = 1;
					NoUse[4]++;
				}
				//Station Insertion Operator Selection //
				r = ((double)rand() / (RAND_MAX + 1));
				if (r < Pr[NSR + 1]) {
					GreedyStationInsertion1();
					Score[NSR + 1] = 1;
					NoUse[NSR + 1]++;
				}
				else if (r > Pr[NSR + 1] && r < Pr[NSR + 2]) {
					GreedyStationInsertionComparison2();
					Score[NSR + 2] = 1;
					NoUse[NSR + 2]++;
				}
				else if (r > Pr[NSR + 2] && r < Pr[NSR + 3]) {
					BestStationInsertion3();
					Score[NSR + 3] = 1;
					NoUse[NSR + 3]++;
				}
			}
			else if ((r > (Value1 / SumValues)) && (r < ((Value1 + Value2) / SumValues))) {   /// remove r <= 1
				ValueR = 1;
				//Route Removal Operator Selection //
				for (int q = 0; q <= RouteIter; q++) {
					r = ((double)rand() / (RAND_MAX + 1));
					if (r <= 0.5) {
						RandomRouteRemoval1();
					}
					else {
						GreedyRouteRemoval2();
					}
					// Customer Insertion Operator Selection //
					r = ((double)rand() / (RAND_MAX + 1));
					if (r < Pr[NSR + NSI + NCR + 1]) {
						GreedyCustomerInsertion1();
						Score[NSR + NSI + NCR + 1] = 1;
						NoUse[NSR + NSI + NCR + 1]++;
					}
					else if (r > Pr[NSR + NSI + NCR + 1] && r < Pr[NSR + NSI + NCR + 2]) {
						RegretCustomerInsertion2();
						Score[NSR + NSI + NCR + 2] = 1;
						NoUse[NSR + NSI + NCR + 2]++;
					}
					else if (r > Pr[NSR + NSI + NCR + 2] && r < Pr[NSR + NSI + NCR + 3]) {
						NoiseCustomerInsertion3();
						Score[NSR + NSI + NCR + 3] = 1;
						NoUse[NSR + NSI + NCR + 3]++;
					}
					for (k = 1; k <= NoRoutes; k++) {
						if (uPtr[Counter[k]][k] > 540) {
							int violation = 1;
						}
						for (i = 1; i <= Counter[k]; i++) {
							if (uPtr[i][k] < 0) {
								int violation = 1;
							}
						}
					}
				}
			}
			else {

				// Customer Removal Operator Selection //
				ValueC = 1;
				PercentRemovalC = 0.4;
				NoRemovalC = ceil(PercentRemovalC * NoCustomers);
				r = ((double)rand() / (RAND_MAX + 1));
				if (r < Pr[NSR + NSI + 1]) {
					RandomCustomerRemoval1();
					Score[NSR + NSI + 1] = 1;
					NoUse[NSR + NSI + 1]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 1] && r < Pr[NSR + NSI + 2]) {
					WorstDistanceRemoval2();
					Score[NSR + NSI + 2] = 1;
					NoUse[NSR + NSI + 2]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 2] && r < Pr[NSR + NSI + 3]) {
					WorstTimeRemoval3();
					Score[NSR + NSI + 3] = 1;
					NoUse[NSR + NSI + 3]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 3] && r < Pr[NSR + NSI + 4]) {
					ShawRemoval4();
					Score[NSR + NSI + 4] = 1;
					NoUse[NSR + NSI + 4]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 4] && r < Pr[NSR + NSI + 5]) {
					ProximityRemoval5();
					Score[NSR + NSI + 5] = 1;
					NoUse[NSR + NSI + 5]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 5] && r < Pr[NSR + NSI + 6]) {
					DemandRemoval6();
					Score[NSR + NSI + 6] = 1;
					NoUse[NSR + NSI + 6]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 6] && r < Pr[NSR + NSI + 7]) {
					TimeBasedRemoval7();
					Score[NSR + NSI + 7] = 1;
					NoUse[NSR + NSI + 7]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 7] && r < Pr[NSR + NSI + 8]) {
					StationVicinityRemoval8();
					Score[NSR + NSI + 8] = 1;
					NoUse[NSR + NSI + 8]++;
					RCwPS();
					RCwSS;
				}
				Infeasibility = 0;
				for (k = 1; k <= NoRoutes; k++) {
					for (i = 1; i <= Counter[k]; i++) {
						if (yPtr[i][k] < 0) {
							Infeasibility = 1;
							break;
						}
					}
				}
				if (RoutedCustomers < NoCustomers) {
					Infeasibility = 1;
				}
				if (Infeasibility == 1) {
					GreedyStationInsertion1();
					// Customer Insertion Operator Selection //
					r = ((double)rand() / (RAND_MAX + 1));
					if (r < Pr[NSR + NSI + NCR + 1]) {
						GreedyCustomerInsertion1();
						Score[NSR + NSI + NCR + 1] = 1;
						NoUse[NSR + NSI + NCR + 1]++;
					}
					else if (r > Pr[NSR + NSI + NCR + 1] && r < Pr[NSR + NSI + NCR + 2]) {
						RegretCustomerInsertion2();
						Score[NSR + NSI + NCR + 2] = 1;
						NoUse[NSR + NSI + NCR + 2]++;
					}
					else if (r > Pr[NSR + NSI + NCR + 2] && r < Pr[NSR + NSI + NCR + 3]) {
						NoiseCustomerInsertion3();
						Score[NSR + NSI + NCR + 3] = 1;
						NoUse[NSR + NSI + NCR + 3]++;
					}
					Infeasibility = 0;
					for (k = 1; k <= NoRoutes; k++) {
						for (i = 1; i <= Counter[k]; i++) {
							if (yPtr[i][k] < 0) {
								Infeasibility = 1;
								break;
							}
						}
					}
					if (Infeasibility == 1) {
						GreedyStationInsertion1();

					}
				}
			}
			// Using SA to accept or reject solution
			// Take care of infeasiblity / Routed Customers
			Infeasibility = 0;
			for (k = 1; k <= NoRoutes; k++) {
				for (i = 1; i <= Counter[k]; i++) {
					if (yPtr[i][k] < 0) {
						Infeasibility = 1;
						break;
					}
				}
			}
			if (RoutedCustomers < NoCustomers) {
				Infeasibility = 1;
			}
			if (NoRoutes <= BestNoRoutes && Emissions > CurrentEmissions && Infeasibility == 0) {
				ProbAcceptance = exp((CurrentEmissions - Emissions) / Temprature);
				Temprature = Temprature * CoolingPar;
				if (ProbAcceptance > 0.5) {
					for (j = 1; j <= NoOperators; j++) {
						if (Score[j] == 1) {
							Score[j] = Score[j] - 1 + Sigma3;
						}
					}
					UpdateCurrent();
					UpdateWeights();
				}
			}
			else if (NoRoutes <= BestNoRoutes && Emissions < BestEmissions && Infeasibility == 0) {
				if (ValueS == 1) {
					Value1 += Beta;
				}
				if (ValueR == 1) {
					Value2 += Beta;
				}
				if (ValueC == 1) {
					Value3 += Beta;
				}
				for (j = 1; j <= NoOperators; j++) {
					if (Score[j] == 1) {
						Score[j] = Score[j] - 1 + Sigma1;
					}
				}
				UpdateBest();
				UpdateWeights();
			}
			else if (NoRoutes <= BestNoRoutes && Emissions < CurrentEmissions && Infeasibility == 0) {
				if (ValueS == 1) {
					Value1 += Beta;
				}
				if (ValueR == 1) {
					Value2 += Beta;
				}
				if (ValueC == 1) {
					Value3 += Beta;
				}
				for (j = 1; j <= NoOperators; j++) {
					if (Score[j] == 1) {
						Score[j] = Score[j] - 1 + Sigma2;
					}
				}
				UpdateCurrent();
				UpdateWeights();
				//}
			}
			Iter++;
			// Update weights of CR and CI algorithms
		}
	}
	EConstraint = BestEmissions + Epsilon;
	SumValues = 0;
	Temprature = 40;
	ProbAcceptance = 0;
	IterO = 0;
	Sigma1 = 15;
	Sigma2 = 10; //10
	Sigma3 = 5; //2 5
	Beta = 10;  //10 3
	Infeasibility = 0;
	s = 0;
	// Start with intital solution
	BestCost = InitialCost;
	CurrentCost = InitialCost;
	BestEmissions = InitialEmissions;
	CurrentEmissions = InitialEmissions;
	BestNoRoutes = InitialNoRoutes;
	CurrentNoRoutes = InitialNoRoutes;
	BestNoEV = InitialNoEV;
	CurrentNoEV = InitialNoEV;
	BestNoCV = InitialNoCV;
	CurrentNoCV = InitialNoCV;
	for (k = 1; k <= NoVehicles; k++) {
		BestEnergyConsumption[k] = InitialEnergyConsumption[k];
		CurrentEnergyConsumption[k] = InitialEnergyConsumption[k];
		BestMaxEnergyCapacity[k] = InitialMaxEnergyCapacity[k];
		CurrentMaxEnergyCapacity[k] = InitialMaxEnergyCapacity[k];
		BestTCost[k] = InitialTCost[k];
		CurrentTCost[k] = InitialTCost[k];
		BestAcquisitionCost[k] = InitialAcquisitionCost[k];
		CurrentAcquisitionCost[k] = InitialAcquisitionCost[k];
	}
	for (k = 1; k <= InitialNoRoutes; k++) {
		BestRouteLength[k] = InitialRouteLength[k];
		CurrentRouteLength[k] = InitialRouteLength[k];
		BestCounter[k] = InitialCounter[k];
		CurrentCounter[k] = InitialCounter[k];

		for (i = 0; i <= InitialCounter[k]; i++) {
			BestRoute[i][k] = InitialRoute[i][k];
			CurrentRoute[i][k] = InitialRoute[i][k];
			BestU[i][k] = InitialU[i][k];
			CurrentU[i][k] = InitialU[i][k];
			BestQ[i][k] = InitialQ[i][k];
			CurrentQ[i][k] = InitialQ[i][k];
			BestY[i][k] = InitialY[i][k];
			CurrentY[i][k] = InitialY[i][k];
		}
	}
	for (k = 1; k <= NoVehicles; k++) {
		BestEnergyConsumption[k] = InitialEnergyConsumption[k];
		CurrentEnergyConsumption[k] = InitialEnergyConsumption[k];
		BestMaxEnergyCapacity[k] = InitialMaxEnergyCapacity[k];
		CurrentMaxEnergyCapacity[k] = InitialMaxEnergyCapacity[k];
		BestTCost[k] = InitialTCost[k];
		CurrentTCost[k] = InitialTCost[k];
		BestAcquisitionCost[k] = InitialAcquisitionCost[k];
		CurrentAcquisitionCost[k] = InitialAcquisitionCost[k];
	}
	for (k = 0; k <= InitialNoRoutes; k++) {
		for (m = 0; m <= NoChargers; m++) {
			for (i = 0; i <= Counter[k]; i++) {
				BestV[i][m][k] = InitialV[i][m][k];
				CurrentV[i][m][k] = InitialV[i][m][k];
			}
		}
	}
	for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
		BestCounterStation[i] = InitialCounterStation[i];
		CurrentCounterStation[i] = InitialCounterStation[i];
	}
	WeightedSum = Weight2nd * InitialCost + (1 - Weight2nd) * InitialEmissions;
	BestWeightedSum = WeightedSum;
	CurrentWeightedSum = WeightedSum;
	// Define weights for the first iteration for Station Removal Operators
	Sum = 0;
	for (i = 1; i <= NSR; i++) { 
		Sum += Weight[s][i];
	}
	for (i = 1; i <= NSR; i++) {
		Probability[s][i] = Weight[s][i] / Sum;
	}
	for (i = 1; i <= NSR; i++) {
		Score[i] = 0;
	}
	Pr[1] = Probability[s][1];
	for (i = 2; i <= NSR; i++) {
		Pr[i] = Pr[i - 1] + Probability[s][i];
	}
	// Define weights for the first iteration for Station Insertion Operators
	Sum = 0;
	for (i = NSR + 1; i <= NSR + NSI; i++) {
		Sum += Weight[s][i];
	}
	for (i = NSR + 1; i <= NSR + NSI; i++) {
		Probability[s][i] = Weight[s][i] / Sum;
	}
	for (i = NSR + 1; i <= NSR + NSI; i++) {
		Score[i] = 0;
	}
	Pr[NSR + 1] = Probability[s][NSR + 1];
	for (i = NSR + 2; i <= NSR + NSI; i++) {
		Pr[i] = Pr[i - 1] + Probability[s][i];
	}
	// Define weights for the first iteration for Customer Removal Operators
	Sum = 0;
	for (i = NSR + NSI + 1; i <= NSR + NSI + NCR; i++) {
		Sum += Weight[s][i];
	}
	for (i = NSR + NSI + 1; i <= NSR + NSI + NCR; i++) {
		Probability[s][i] = Weight[s][i] / Sum;
	}
	for (i = NSR + NSI + 1; i <= NSR + NSI + NCR; i++) {
		Score[i] = 0;
	}
	Pr[NSR + NSI + 1] = Probability[s][NSR + NSI + 1];
	for (i = NSR + NSI + 2; i <= NSR + NSI + NCR; i++) {
		Pr[i] = Pr[i - 1] + Probability[s][i];
	}
	// Define weights for the first iteration for Customer Insertion Operators
	Sum = 0;
	for (i = NSR + NSI + NCR + 1; i <= NSR + NSI + NCR + NCI; i++) {
		Sum += Weight[s][i];
	}
	for (i = NSR + NSI + NCR + 1; i <= NSR + NSI + NCR + NCI; i++) {
		Probability[s][i] = Weight[s][i] / Sum;
	}
	for (i = NSR + NSI + NCR + 1; i <= NSR + NSI + NCR + NCI; i++) {
		Score[i] = 0;
	}
	Pr[NSR + NSI + NCR + 1] = Probability[s][NSR + NSI + NCR + 1];
	for (i = NSR + NSI + NCR + 2; i <= NSR + NSI + NCR + NCI; i++) {
		Pr[i] = Pr[i - 1] + Probability[s][i];
	}
	Value1 = Value2 = Value3 = 1;

	while (IterO < OuterIter) {
		Iter = 0;
		IterO++;
		// Should I define a new function for this??
		Cost = BestCost;
		Emissions = BestEmissions;
		WeightedSum = Weight2nd * Cost + (1 - Weight2nd) * Emissions;
		NoEV = BestNoEV;
		NoCV = BestNoCV;
		for (k = 1; k <= NoRoutes; k++) {
			for (i = 0; i <= Counter[k]; i++) {
				Route[i][k] = 0;
				uPtr[i][k] = 0;
				qPtr[i][k] = 0;
				yPtr[i][k] = 0;
			}
			RouteLength[k] = 0;
		}
		for (k = 0; k <= NoRoutes; k++) {
			for (m = 0; m <= NoChargers; m++) {
				for (i = 0; i <= Counter[k]; i++) {
					vPtr[i][m][k] = 0;
				}
			}
			Counter[k] = 0;

		}
		NoRoutes = BestNoRoutes;
		for (k = 1; k <= NoVehicles; k++) {
			MaxEnergyCapacity[k] = BestMaxEnergyCapacity[k];
			TCost[k] = BestTCost[k];
			EnergyConsumption[k] = BestEnergyConsumption[k];
			AcquisitionCost[k] = BestAcquisitionCost[k];
		}
		for (k = 1; k <= BestNoRoutes; k++) {
			RouteLength[k] = BestRouteLength[k];
			Counter[k] = BestCounter[k];
			for (i = 0; i <= Counter[k]; i++) {
				Route[i][k] = BestRoute[i][k];
				uPtr[i][k] = BestU[i][k];
				qPtr[i][k] = BestQ[i][k];
				yPtr[i][k] = BestY[i][k];
			}
		}

		for (k = 0; k <= NoRoutes; k++) {
			for (m = 0; m <= NoChargers; m++) {
				for (i = 0; i <= Counter[k]; i++) {
					vPtr[i][m][k] = BestV[i][m][k];
				}
			}
		}
		for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
			CounterStation[i] = 0;
		}
		for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
			CounterStation[i] = BestCounterStation[i];
		}
		for (i = 0; i <= NoCustomers; i++) {
			RemovedCustomer[i] = 0;
		}
		CounterRemoval = 0;
		RoutedCustomers = NoCustomers;
		//Swap();
		Value1 = Value2 = Value3 = 1;
		while (Iter <= InnerIter) {
			OperatorCounter = 0;
			//Random+=114; //(123/114/172/114)
			//srand(Random);
			ValueS = ValueC = ValueR = 0;
			for (i = 1; i <= NSR + NSI + NCR + NCI; i++) {
				Score[i] = 0;
			}
			for (k = 1; k <= NoRoutes; k++) {
				if (uPtr[Counter[k]][k] > 540) {
					int violation = 1;
				}
				for (i = 1; i <= Counter[k]; i++) {
					if (uPtr[i][k] < 0) {
						int violation = 1;
					}
				}
			}
			SumValues = Value1 + Value2 + Value3;
			r = ((double)rand() / (RAND_MAX + 1));
			for (k = 1; k <= BestNoRoutes; k++) {
				for (i = 0; i <= BestCounter[k]; i++) {
					if (BestU[i][k] < 0) {
						int Violation = 1;
					}
				}
			}
			for (k = 1; k <= BestNoRoutes; k++) {
				if (BestU[BestCounter[k]][k] > 540) {
					int Violation = 1;
				}
			}
			if (r < (Value1 / SumValues)) {
				ValueS = 1;
				//Station Removal Operator Selection //
				r = ((double)rand() / (RAND_MAX + 1));
				if (r < Pr[1]) {
					RandomStationRemoval1();
					Score[1] = 1;
					NoUse[1]++;
				}
				else if (r > Pr[1] && r < Pr[2]) {
					WorstDistanceStationRemoval2();
					Score[2] = 1;
					NoUse[2]++;
				}
				else if (r > Pr[2] && r < Pr[3]) {
					LeastUsedStationRemoval3();
					Score[3] = 1;
					NoUse[3]++;
				}
				else if (r > Pr[3] && r < Pr[4]) {
					ExpensiveStationRemoval4();
					Score[4] = 1;
					NoUse[4]++;
				}
				//Station Insertion Operator Selection //
				r = ((double)rand() / (RAND_MAX + 1));
				if (r < Pr[NSR + 1]) {
					GreedyStationInsertion1();
					Score[NSR + 1] = 1;
					NoUse[NSR + 1]++;
				}
				else if (r > Pr[NSR + 1] && r < Pr[NSR + 2]) {
					GreedyStationInsertionComparison2();
					Score[NSR + 2] = 1;
					NoUse[NSR + 2]++;
				}
				else if (r > Pr[NSR + 2] && r < Pr[NSR + 3]) {
					BestStationInsertion3();
					Score[NSR + 3] = 1;
					NoUse[NSR + 3]++;
				}
			}
			else if ((r > (Value1 / SumValues)) && (r < ((Value1 + Value2) / SumValues))) {   /// remove r <= 1
				ValueR = 1;
				//Route Removal Operator Selection //
				for (int q = 0; q <= RouteIter; q++) {
					r = ((double)rand() / (RAND_MAX + 1));
					if (r <= 0.5) {
						RandomRouteRemoval1();
					}
					else {
						GreedyRouteRemoval2();
					}
					// Customer Insertion Operator Selection //
					r = ((double)rand() / (RAND_MAX + 1));
					if (r < Pr[NSR + NSI + NCR + 1]) {
						GreedyCustomerInsertion1();
						Score[NSR + NSI + NCR + 1] = 1;
						NoUse[NSR + NSI + NCR + 1]++;
					}
					else if (r > Pr[NSR + NSI + NCR + 1] && r < Pr[NSR + NSI + NCR + 2]) {
						RegretCustomerInsertion2();
						Score[NSR + NSI + NCR + 2] = 1;
						NoUse[NSR + NSI + NCR + 2]++;
					}
					else if (r > Pr[NSR + NSI + NCR + 2] && r < Pr[NSR + NSI + NCR + 3]) {
						NoiseCustomerInsertion3();
						Score[NSR + NSI + NCR + 3] = 1;
						NoUse[NSR + NSI + NCR + 3]++;
					}
					for (k = 1; k <= NoRoutes; k++) {
						if (uPtr[Counter[k]][k] > 540) {
							int violation = 1;
						}
						for (i = 1; i <= Counter[k]; i++) {
							if (uPtr[i][k] < 0) {
								int violation = 1;
							}
						}
					}
				}
			}
			else {

				// Customer Removal Operator Selection //
				ValueC = 1;
				PercentRemovalC = 0.4;
				NoRemovalC = ceil(PercentRemovalC * NoCustomers);
				r = ((double)rand() / (RAND_MAX + 1));
				if (r < Pr[NSR + NSI + 1]) {
					RandomCustomerRemoval1();
					Score[NSR + NSI + 1] = 1;
					NoUse[NSR + NSI + 1]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 1] && r < Pr[NSR + NSI + 2]) {
					WorstDistanceRemoval2();
					Score[NSR + NSI + 2] = 1;
					NoUse[NSR + NSI + 2]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 2] && r < Pr[NSR + NSI + 3]) {
					WorstTimeRemoval3();
					Score[NSR + NSI + 3] = 1;
					NoUse[NSR + NSI + 3]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 3] && r < Pr[NSR + NSI + 4]) {
					ShawRemoval4();
					Score[NSR + NSI + 4] = 1;
					NoUse[NSR + NSI + 4]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 4] && r < Pr[NSR + NSI + 5]) {
					ProximityRemoval5();
					Score[NSR + NSI + 5] = 1;
					NoUse[NSR + NSI + 5]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 5] && r < Pr[NSR + NSI + 6]) {
					DemandRemoval6();
					Score[NSR + NSI + 6] = 1;
					NoUse[NSR + NSI + 6]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 6] && r < Pr[NSR + NSI + 7]) {
					TimeBasedRemoval7();
					Score[NSR + NSI + 7] = 1;
					NoUse[NSR + NSI + 7]++;
					RCwPS();
					RCwSS;
				}
				else if (r > Pr[NSR + NSI + 7] && r < Pr[NSR + NSI + 8]) {
					StationVicinityRemoval8();
					Score[NSR + NSI + 8] = 1;
					NoUse[NSR + NSI + 8]++;
					RCwPS();
					RCwSS;
				}
				Infeasibility = 0;
				for (k = 1; k <= NoRoutes; k++) {
					for (i = 1; i <= Counter[k]; i++) {
						if (yPtr[i][k] < 0) {
							Infeasibility = 1;
							break;
						}
					}
				}
				if (RoutedCustomers < NoCustomers) {
					Infeasibility = 1;
				}
				if (Infeasibility == 1) {
					GreedyStationInsertion1();
					// Customer Insertion Operator Selection //
					r = ((double)rand() / (RAND_MAX + 1));
					if (r < Pr[NSR + NSI + NCR + 1]) {
						GreedyCustomerInsertion1();
						Score[NSR + NSI + NCR + 1] = 1;
						NoUse[NSR + NSI + NCR + 1]++;
					}
					else if (r > Pr[NSR + NSI + NCR + 1] && r < Pr[NSR + NSI + NCR + 2]) {
						RegretCustomerInsertion2();
						Score[NSR + NSI + NCR + 2] = 1;
						NoUse[NSR + NSI + NCR + 2]++;
					}
					else if (r > Pr[NSR + NSI + NCR + 2] && r < Pr[NSR + NSI + NCR + 3]) {
						NoiseCustomerInsertion3();
						Score[NSR + NSI + NCR + 3] = 1;
						NoUse[NSR + NSI + NCR + 3]++;
					}
					Infeasibility = 0;
					for (k = 1; k <= NoRoutes; k++) {
						for (i = 1; i <= Counter[k]; i++) {
							if (yPtr[i][k] < 0) {
								Infeasibility = 1;
								break;
							}
						}
					}
					if (Infeasibility == 1) {
						GreedyStationInsertion1();

					}
				}
			}
			// Using SA to accept or reject solution
			// Take care of infeasiblity / Routed Customers
			Infeasibility = 0;
			for (k = 1; k <= NoRoutes; k++) {
				for (i = 1; i <= Counter[k]; i++) {
					if (yPtr[i][k] < 0) {
						Infeasibility = 1;
						break;
					}
				}
			}
			if (RoutedCustomers < NoCustomers) {
				Infeasibility = 1;
			}
			for (k = 1; k <= NoRoutes; k++) {
				for (i = 0; i <= Counter[k] - 1; i++) {
					if (MaxEnergyCapacity[k] > EnergyLimit) {
						Emissions += (Lambda * CurbWeight * Rho * Sigma2nd * (d[Route[i][k]][Route[i + 1][k]])); 
						Emissions += (Lambda * EngineFriction * EngineSpeed * EngineDisplacement * (d[Route[i][k]][Route[i + 1][k]]/speed));
						Emissions += (Lambda * Epsilon2nd * Rho * d[Route[i][k]][Route[i + 1][k]] * speed * speed);
						Emissions += (Rho * Sigma2nd * Lambda * (d[Route[i][k]][Route[i + 1][k]]) * qPtr[i][k]);
					}
				}
			}
			WeightedSum = Weight2nd * Cost + (1 - Weight2nd) * Emissions;
			if (NoRoutes <= BestNoRoutes && Emissions < EConstraint && WeightedSum > CurrentWeightedSum && Infeasibility == 0) {
				ProbAcceptance = exp((CurrentWeightedSum - WeightedSum) / Temprature);
				Temprature = Temprature * CoolingPar;
				if (ProbAcceptance > 0.5) { ////needs to be a random number
					for (j = 1; j <= NoOperators; j++) {
						if (Score[j] == 1) {
							Score[j] = Score[j] - 1 + Sigma3;
						}
					}
					UpdateCurrent();
					UpdateWeights();
				}
			}
			else if (NoRoutes <= BestNoRoutes && Emissions < EConstraint && WeightedSum < BestWeightedSum && Infeasibility == 0) {
				if (ValueS == 1) {
					Value1 += Beta;
				}
				if (ValueR == 1) {
					Value2 += Beta;
				}
				if (ValueC == 1) {
					Value3 += Beta;
				}
				for (j = 1; j <= NoOperators; j++) {
					if (Score[j] == 1) {
						Score[j] = Score[j] - 1 + Sigma1;
					}
				}
				UpdateBest();
				UpdateWeights();
			}
			else if (NoRoutes <= BestNoRoutes && Emissions < EConstraint && WeightedSum < CurrentWeightedSum && Infeasibility == 0) {
				if (ValueS == 1) {
					Value1 += Beta;
				}
				if (ValueR == 1) {
					Value2 += Beta;
				}
				if (ValueC == 1) {
					Value3 += Beta;
				}
				for (j = 1; j <= NoOperators; j++) {
					if (Score[j] == 1) {
						Score[j] = Score[j] - 1 + Sigma2;
					}
				}
				UpdateCurrent();
				UpdateWeights();
				//}
			}
			Iter++;
			// Update weights of CR and CI algorithms
		}
		if (IterO == BiIter) {
			Cost = BestCost;
			Emissions = BestEmissions;
			WeightedSum = BestWeightedSum;
			NoEV = BestNoEV;
			NoCV = BestNoCV;
			for (k = 1; k <= NoRoutes; k++) {
				for (i = 0; i <= Counter[k]; i++) {
					Route[i][k] = 0;
					uPtr[i][k] = 0;
					qPtr[i][k] = 0;
					yPtr[i][k] = 0;
				}
				RouteLength[k] = 0;
			}
			for (k = 0; k <= NoRoutes; k++) {
				for (m = 0; m <= NoChargers; m++) {
					for (i = 0; i <= Counter[k]; i++) {
						vPtr[i][m][k] = 0;
					}
				}
				Counter[k] = 0;

			}
			NoRoutes = BestNoRoutes;
			for (k = 1; k <= NoVehicles; k++) {
				MaxEnergyCapacity[k] = BestMaxEnergyCapacity[k];
				TCost[k] = BestTCost[k];
				EnergyConsumption[k] = BestEnergyConsumption[k];
				AcquisitionCost[k] = BestAcquisitionCost[k];
			}
			for (k = 1; k <= BestNoRoutes; k++) {
				RouteLength[k] = BestRouteLength[k];
				Counter[k] = BestCounter[k];
				for (i = 0; i <= Counter[k]; i++) {
					Route[i][k] = BestRoute[i][k];
					uPtr[i][k] = BestU[i][k];
					qPtr[i][k] = BestQ[i][k];
					yPtr[i][k] = BestY[i][k];
				}
			}

			for (k = 0; k <= NoRoutes; k++) {
				for (m = 0; m <= NoChargers; m++) {
					for (i = 0; i <= Counter[k]; i++) {
						vPtr[i][m][k] = BestV[i][m][k];
					}
				}
			}
			for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
				CounterStation[i] = 0;
			}
			for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
				CounterStation[i] = BestCounterStation[i];
			}
			for (i = 0; i <= NoCustomers; i++) {
				RemovedCustomer[i] = 0;
			}
			CounterRemoval = 0;
			RoutedCustomers = NoCustomers;
			BiIter += 20;
			Epsilon += 2;
			Weight2nd += 0.1;
			EConstraint += Epsilon;
			NoSolution++;
			for (k = 1; k <= BestNoRoutes; k++) {
				for (i = 1; i <= BestCounter[k] - 1; i++) {
					for (m = 1; m <= NoChargers; m++) {
						if (BestV[i][m][k] > 0) {
							NoLevel3++;
						}
					}
				}
			}
			////////////////
			for (k = 1; k <= BestNoRoutes; k++) {
				if (BestRoute[1][k] != 0) {
					if (BestMaxEnergyCapacity[k] < EnergyLimit) {
						EVLength += BestRouteLength[k];
					}
					else {
						CVLength += BestRouteLength[k];
					}
				}
			}
			Pareto[NoSolution] = BestWeightedSum;
			FirstObj[NoSolution] = BestCost;
			SecondObj[NoSolution] = BestEmissions;
			ParetoNoEV[NoSolution] = BestNoEV;
			ParetoNoCV[NoSolution] = BestNoEV;
			ParetoEVLength[NoSolution] = EVLength;
			ParetoCVLength[NoSolution] = CVLength;
			ParetoNoChargers[NoSolution] = NoLevel3;
			// Start with intial solution
			BestCost = InitialCost;
			CurrentCost = InitialCost;
			BestEmissions = InitialEmissions;
			CurrentEmissions = InitialEmissions;
			BestWeightedSum = Weight2nd * InitialCost + (1 - Weight2nd) * InitialEmissions;
			CurrentWeightedSum = Weight2nd * InitialCost + (1 - Weight2nd) * InitialEmissions;
			BestNoRoutes = InitialNoRoutes;
			CurrentNoRoutes = InitialNoRoutes;
			BestNoEV = InitialNoEV;
			CurrentNoEV = InitialNoEV;
			BestNoCV = InitialNoCV;
			CurrentNoCV = InitialNoCV;
			for (k = 1; k <= NoVehicles; k++) {
				BestEnergyConsumption[k] = InitialEnergyConsumption[k];
				CurrentEnergyConsumption[k] = InitialEnergyConsumption[k];
				BestMaxEnergyCapacity[k] = InitialMaxEnergyCapacity[k];
				CurrentMaxEnergyCapacity[k] = InitialMaxEnergyCapacity[k];
				BestTCost[k] = InitialTCost[k];
				CurrentTCost[k] = InitialTCost[k];
				BestAcquisitionCost[k] = InitialAcquisitionCost[k];
				CurrentAcquisitionCost[k] = InitialAcquisitionCost[k];
			}
			for (k = 1; k <= InitialNoRoutes; k++) {
				BestRouteLength[k] = InitialRouteLength[k];
				CurrentRouteLength[k] = InitialRouteLength[k];
				BestCounter[k] = InitialCounter[k];
				CurrentCounter[k] = InitialCounter[k];

				for (i = 0; i <= InitialCounter[k]; i++) {
					BestRoute[i][k] = InitialRoute[i][k];
					CurrentRoute[i][k] = InitialRoute[i][k];
					BestU[i][k] = InitialU[i][k];
					CurrentU[i][k] = InitialU[i][k];
					BestQ[i][k] = InitialQ[i][k];
					CurrentQ[i][k] = InitialQ[i][k];
					BestY[i][k] = InitialY[i][k];
					CurrentY[i][k] = InitialY[i][k];
				}
			}
			for (k = 1; k <= NoVehicles; k++) {
				BestEnergyConsumption[k] = InitialEnergyConsumption[k];
				CurrentEnergyConsumption[k] = InitialEnergyConsumption[k];
				BestMaxEnergyCapacity[k] = InitialMaxEnergyCapacity[k];
				CurrentMaxEnergyCapacity[k] = InitialMaxEnergyCapacity[k];
				BestTCost[k] = InitialTCost[k];
				CurrentTCost[k] = InitialTCost[k];
				BestAcquisitionCost[k] = InitialAcquisitionCost[k];
				CurrentAcquisitionCost[k] = InitialAcquisitionCost[k];
			}
			for (k = 0; k <= InitialNoRoutes; k++) {
				for (m = 0; m <= NoChargers; m++) {
					for (i = 0; i <= Counter[k]; i++) {
						BestV[i][m][k] = InitialV[i][m][k];
						CurrentV[i][m][k] = InitialV[i][m][k];
					}
				}
			}
			for (i = NoCustomers + 1; i <= NoCustomers + NoStations; i++) {
				BestCounterStation[i] = InitialCounterStation[i];
				CurrentCounterStation[i] = InitialCounterStation[i];
			}

			// Define weights for the first iteration for Station Removal Operators
			Sum = 0;
			for (i = 1; i <= NSR; i++) {
				Sum += Weight[s][i];
			}
			for (i = 1; i <= NSR; i++) {
				Probability[s][i] = Weight[s][i] / Sum;
			}
			for (i = 1; i <= NSR; i++) {
				Score[i] = 0;
			}
			Pr[1] = Probability[s][1];
			for (i = 2; i <= NSR; i++) {
				Pr[i] = Pr[i - 1] + Probability[s][i];
			}
			// Define weights for the first iteration for Station Insertion Operators
			Sum = 0;
			for (i = NSR + 1; i <= NSR + NSI; i++) {
				Sum += Weight[s][i];
			}
			for (i = NSR + 1; i <= NSR + NSI; i++) {
				Probability[s][i] = Weight[s][i] / Sum;
			}
			for (i = NSR + 1; i <= NSR + NSI; i++) {
				Score[i] = 0;
			}
			Pr[NSR + 1] = Probability[s][NSR + 1];
			for (i = NSR + 2; i <= NSR + NSI; i++) {
				Pr[i] = Pr[i - 1] + Probability[s][i];
			}
			// Define weights for the first iteration for Customer Removal Operators
			Sum = 0;
			for (i = NSR + NSI + 1; i <= NSR + NSI + NCR; i++) {
				Sum += Weight[s][i];
			}
			for (i = NSR + NSI + 1; i <= NSR + NSI + NCR; i++) {
				Probability[s][i] = Weight[s][i] / Sum;
			}
			for (i = NSR + NSI + 1; i <= NSR + NSI + NCR; i++) {
				Score[i] = 0;
			}
			Pr[NSR + NSI + 1] = Probability[s][NSR + NSI + 1];
			for (i = NSR + NSI + 2; i <= NSR + NSI + NCR; i++) {
				Pr[i] = Pr[i - 1] + Probability[s][i];
			}
			// Define weights for the first iteration for Customer Insertion Operators
			Sum = 0;
			for (i = NSR + NSI + NCR + 1; i <= NSR + NSI + NCR + NCI; i++) {
				Sum += Weight[s][i];
			}
			for (i = NSR + NSI + NCR + 1; i <= NSR + NSI + NCR + NCI; i++) {
				Probability[s][i] = Weight[s][i] / Sum;
			}
			for (i = NSR + NSI + NCR + 1; i <= NSR + NSI + NCR + NCI; i++) {
				Score[i] = 0;
			}
			Pr[NSR + NSI + NCR + 1] = Probability[s][NSR + NSI + NCR + 1];
			for (i = NSR + NSI + NCR + 2; i <= NSR + NSI + NCR + NCI; i++) {
				Pr[i] = Pr[i - 1] + Probability[s][i];
			}
			Value1 = Value2 = Value3 = 1;
		}
	}
	ALNSEndTime = second();
	fprintf(Out, "%s%s", "Pareto", "\n");
	fprintf(Out, "%s%s", "FirstObj", "\n");
	fprintf(Out, "%s%s", "SecondObj", "\n");
	fprintf(Out, "%s%s", "ParetoNoEV", "\n");
	fprintf(Out, "%s%s", "ParetoNoCV", "\n");
	fprintf(Out, "%s%s", "ParetoNoChargers", "\n");
	//fprintf(Out, "%s%s", "ParetoEVLength", "\n");
	//fprintf(Out, "%s%s", "ParetoCVLength", "\n");
	for (i = 1; i <= 10; i++) {
		fprintf(Out, "%d%s", i , "	");
		fprintf(Out, "%f%s", Pareto[i], "	");
		fprintf(Out, "%f%s", FirstObj[i], "		");
		fprintf(Out, "%f%s", SecondObj[i], "	");
		fprintf(Out, "%d%s", ParetoNoEV[i], "	");
		fprintf(Out, "%d%s", ParetoNoCV[i], "	");
		fprintf(Out, "%d%s", ParetoNoChargers[i], "	");
		//fprintf(Out, "%f%s", ParetoEVLength[i], "	");
		//fprintf(Out, "%f%s", ParetoCVLength[i], "	");
		fprintf(Out, "%s", "\n");
	}
	fprintf(Out, "%s%f%s", "Final: Time = ", ALNSEndTime - ALNSStartTime, "\n");

	//////// Delete ///////

	/*if (Xcoordinate != NULL) {
		delete[] Xcoordinate;
	}*/
	if (Ycoordinate != NULL) {
		delete[] Ycoordinate;
	}
	if (Demand != NULL) {
		delete[] Demand;
	}
	if (eTime != NULL) {
		delete[] eTime;
	}
	if (lTime != NULL) {
		delete[] lTime;
	}
	if (ServiceTime != NULL) {
		delete[] ServiceTime;
	}
	if (fCost != NULL) {
		delete[] fCost;
	}
	if (TCost != NULL) {
		delete[] TCost;
	}
	if (BestTCost != NULL) {
		delete[] BestTCost;
	}
	if (CurrentTCost != NULL) {
		delete[] CurrentTCost;
	}
	if (AcquisitionCost != NULL) {
		delete[] AcquisitionCost;
	}
	if (BestAcquisitionCost != NULL) {
		delete[] BestAcquisitionCost;
	}
	if (CurrentAcquisitionCost != NULL) {
		delete[] CurrentAcquisitionCost;
	}
	if (MaxCapacity != NULL) {
		delete[] MaxCapacity;
	}
	if (MaxEnergyCapacity != NULL) {
		delete[] MaxEnergyCapacity;
	}
	if (BestMaxEnergyCapacity != NULL) {
		delete[] BestMaxEnergyCapacity;
	}
	if (CurrentMaxEnergyCapacity != NULL) {
		delete[] CurrentMaxEnergyCapacity;
	}
	if (RCost != NULL) {
		delete[] RCost;
	}

	if (Rate != NULL) {
		delete[] Rate;
	}

	if (EnergyConsumption != NULL) {
		delete[] EnergyConsumption;
	}
	if (BestEnergyConsumption != NULL) {
		delete[] BestEnergyConsumption;
	}
	if (CurrentEnergyConsumption != NULL) {
		delete[] CurrentEnergyConsumption;
	}
	if (Score != NULL) {
		delete[] Score;
	}
	if (NoUse != NULL) {
		delete[] NoUse;
	}
	if (Weight != NULL) {
		for (j = 0; j <= IterS + IterC; j++)
			delete[] Weight[j];
		delete[] Weight;
	}
	if (Pr != NULL) {
		delete[] Pr;
	}
	if (Probability != NULL) {
		for (j = 0; j <= IterS + IterC; j++)
			delete[] Probability[j];
		delete[] Probability;
	}
	if (vPtr != NULL) {
		for (i = 0; i <= NoVertices ; i++) {
			for (j = 0; j <= NoChargers; j++)
				delete[] vPtr[i][j];
			delete[] vPtr[i];
		}
		delete[] vPtr;
	}
	
	if (yPtr != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] yPtr[i];
		delete[] yPtr;
	}
	if (uPtr != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] uPtr[i];
		delete[] uPtr;
	}
	if (SortI != NULL) {
		delete[] SortI;
	}
	if (SortII != NULL) {
		delete[] SortII;
	}
	/*if (SortIII != NULL) {
		delete[] SortIII;
	}*/
	if (Seed != NULL) {
		delete[] Seed;
	}
	if (RouteLength != NULL) {
		delete[] RouteLength;
	}
	if (BestRouteLength != NULL) {
		delete[] BestRouteLength;
	}
	if (CurrentRouteLength != NULL) {
		delete[] CurrentRouteLength;
	}
	if (Route != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] Route[i];
		delete[] Route;
	}
	if (CurrentRoute != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] CurrentRoute[i];
		delete[] CurrentRoute;
	}
	if (BestRoute != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] BestRoute[i];
		delete[] BestRoute;
	}

	/*if (Penalty != NULL) {
		delete[] Penalty;
	}*/
	if (Counter != NULL) {
		delete[] Counter;
	}
	if (CurrentCounter != NULL) {
		delete[] CurrentCounter;
	}
	if (BestCounter != NULL) {
		delete[] BestCounter;
	}
	if (CounterStation != NULL) {
		delete[] CounterStation;
	}
	if (BestCounterStation != NULL) {
		delete[] BestCounterStation;
	}
	if (CurrentCounterStation != NULL) {
		delete[] CurrentCounterStation;
	}
	if (Technology != NULL) {
		delete[] Technology;
	}
	if (RemovedCustomer != NULL) {
		delete[] RemovedCustomer;
	}
	if (RemovalStation != NULL) {
		delete[] RemovalStation;
	}
	if (RemovalRoute != NULL) {
		delete[] RemovalRoute;
	}
	if (TempV != NULL) {
		for (i = 0; i <= NoVertices ; i++) {
			for (j = 0; j <= NoChargers; j++)
				delete[] TempV[i][j];
			delete[] TempV[i];
		}
		delete[] TempV;
	}
	if (TempQ != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] TempQ[i];
		delete[] TempQ;
	}
	if (TempY != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] TempY[i];
		delete[] TempY;
	}
	if (TempU != NULL) {
		delete[] TempU;
	}
	if (CurrentV != NULL) {
		for (i = 0; i <= NoVertices ; i++) {
			for (j = 0; j <= NoChargers; j++)
				delete[] CurrentV[i][j];
			delete[] CurrentV[i];
		}
		delete[] CurrentV;
	}

	if (CurrentQ != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] CurrentQ[i];
		delete[] CurrentQ;
	}
	if (CurrentY != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] CurrentY[i];
		delete[] CurrentY;
	}
	if (CurrentU != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] CurrentU[i];
		delete[] CurrentU;
	}
	if (BestV != NULL) {
		for (i = 0; i <= NoVertices ; i++) {
			for (j = 0; j <= NoChargers; j++)
				delete[] BestV[i][j];
			delete[] BestV[i];
		}
		delete[] BestV;
	}

	if (BestQ != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] BestQ[i];
		delete[] BestQ;
	}
	if (BestY != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] BestY[i];
		delete[] BestY;
	}
	if (BestU != NULL) {
		for (i = 0; i <= NoVertices ; i++)
			delete[] BestU[i];
		delete[] BestU;
	}
	if (t != NULL) {
		for (i = 0; i <= NoVertices; i++)
			delete[] t[i];
		delete[] t;
	}
	if (d != NULL) {
		for (i = 0; i <= NoVertices; i++)
			delete[] d[i];
		delete[] d;
	}
}

void main(void) {
	int i, j, k, l;
	for (int i = 6; i <= 6; i++) { //1-2
		for (j = 3; j <= 3; j++) { //1
			for (k = 1; k <= 1; k++) {//1-3         
				for (l = 1; l <= 1; l++) {//1- 29 2-35 3 -28
					//No+=389; // 124*(36) 1789*(37) 1789*(42) 389*(38) (124/389/108/23)
					sprintf_s(FileName, "%s%d-%d-%d-%d", "C:\\PhD\\Paper 2\\Samples\\sample", i, j, k, l);
					printf_s("%s\n", FileName);
					strcat_s(FileName, ".txt");
					errno_t err = fopen_s(&Input, FileName, "r");
					ReadData(i, j, k);
					InitialSolutionI(i, j, k, l);
					ALNSStartTime = second();
					ALNS();

				}
			}
		}
		//getch();
	}

}