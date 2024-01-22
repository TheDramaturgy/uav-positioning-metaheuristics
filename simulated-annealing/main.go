package main

import (
	"fmt"
	"os"
	"sync"
	"time"

	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/device"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/gateway"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/problem"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/solver"
)

var mainWG = &sync.WaitGroup{}

func main() {
	// ---------- Parse Arguments
	args := os.Args[1:]
	seed := args[0]
	numDevices := args[1]
	numGateways := args[2]
	// prefix := args[3]

	// ---------- Files path names
	cwd, err := os.Getwd()
	if err != nil {
		panic(err)
	}

	devicePositionFile := cwd + "/data/endDevices_LNM_Placement_" + seed + "s+" + numDevices + "d.dat"
	sliceAssociationFile := cwd + "/data/skl_" + seed + "s_" + numGateways + "x1Gv_" + numDevices + "D.dat"
	gatewayPositionFile := cwd + "/data/equidistantPlacement_" + numGateways + ".dat"

	// ---------- Load Data
	deviceList := device.ReadDeviceList(devicePositionFile, sliceAssociationFile)
	fmt.Printf("Successfully loaded %d devices\n", deviceList.Count())

	candidatePosList := gateway.ReadCandidatePositionList(gatewayPositionFile)
	fmt.Printf("Successfully loaded %d candidate positions\n", candidatePosList.Count())

	gw := &gateway.Gateway{}
	gw.SetSensitivity(map[int16]float32{7: -130.0, 8: -132.5, 9: -135.0, 10: -137.5, 11: -140.0, 12: -142.5})
	for slice := range deviceList.Slices() {
		gw.AddSlice(int32(slice), 125000.0, 15197.75390625)
	}

	// ---------- Create problem instance
	instance, err := problem.CreateUAVProblemInstance(100.0, 1.0, 0.0, 0.0, deviceList, candidatePosList, gw)
	if err != nil {
		panic(err)
	}

	// ---------- Solve problem
	//SASolve(instance)
	//TSSolve(instance, seed, numDevices, numGateways, prefix)

	//GRASPSolve(deviceList, candidatePosList, gw, seed, strconv.Itoa(numDevs), numGateways)

	//for i := 0; i < 30; i++ {
	//	//prefix := fmt.Sprintf("random%d", i)
	//	iterations := 25000
	//	population := 50
	//	//crossRate := 0.1
	//	mutationRate := 0.0001
	//
	//	GASolve(instance, iterations, population, 0.7, mutationRate, fmt.Sprintf("ga+grasp%d", i), seed, numGateways, numDevices)
	//}

	filePrefix := "GA"
	iterations := 15000
	population := 50
	crossRate := 0.6
	mutationRate := 0.0001

	GASolve(instance, iterations, population, crossRate, mutationRate, filePrefix, seed, numGateways, numDevices)

}

func GASolve(instance *problem.UAVProblem, it, population int, crossRate, mutationRate float64, prefix, seed, numGateways, numDevices string) {
	cwd, err := os.Getwd()
	if err != nil {
		panic(err)
	}

	//logFile := cwd + "/output/" + prefix + "_perf_solutions.dat"
	//file, err := os.OpenFile(logFile, os.O_APPEND|os.O_WRONLY|os.O_CREATE, 0644)
	//if err != nil {
	//	panic(err)
	//}
	//defer file.Close()

	// file.WriteString("seed,numVirtualPositions,numDevices,numGens,objective\n")
	s := solver.CreateGASolver(instance, it, population, 100, crossRate, mutationRate)
	s.Solve()

	//file.WriteString(fmt.Sprintf("%s,%s,%s,%d,%f,%f\n", seed, numGateways, numDevices, s.GetCurrentIteration(), s.GetBestCostTime(), sol.GetCost()))

	// ---------- Save Result

	logFile := cwd + "/output/" + prefix + "_log_" + seed + "s_" + numGateways + "g_" + numDevices + "d.dat"
	placementFile := cwd + "/output/" + prefix + "_Placement_" + seed + "s_" + numGateways + "x1Gv_" + numDevices + "D.dat"
	configurationFile := cwd + "/output/" + prefix + "_DevicesConfigurations_" + seed + "s_" + numGateways + "x1Gv_" + numDevices + "D.dat"
	ExportResults(s, instance, logFile, placementFile, configurationFile)
}

func GRASPSolve(deviceList *device.DeviceList, candidatePosList *gateway.CandidatePositionList, gw *gateway.Gateway, seed, numDevices, numGateways string) {
	cwd, err := os.Getwd()
	if err != nil {
		panic(err)
	}

	logFile := cwd + "/output/GRASP_perf_solutions.dat"
	file, err := os.OpenFile(logFile, os.O_APPEND|os.O_WRONLY|os.O_CREATE, 0644)
	if err != nil {
		panic(err)
	}
	defer file.Close()

	for i := 0; i < 30; i++ {
		instance, err := problem.CreateUAVProblemInstance(100.0, 1.0, 0.75, 0.001, deviceList, candidatePosList, gw)
		if err != nil {
			panic(err)
		}

		s := solver.CreateGRASPSolver(instance)
		sol := s.Solve()

		file.WriteString(fmt.Sprintf("%s,%s,%s,%d,%f,%d,%f\n", seed, numGateways, numDevices, s.GetCurrentIteration(), s.GetBestCostTime(), len(sol.GetDeployedUavs()), sol.GetCost()))
	}

	// ---------- Save Result

	// prefix := fmt.Sprintf("grasp%d", i)

	// logFile := cwd + "/output/" + prefix + "_log_" + seed + "s_" + numGateways + "g_" + numDevices + "d.dat"
	// placementFile := cwd + "/output/" + prefix + "_Placement_" + seed + "s_" + numGateways + "x1Gv_" + numDevices + "D.dat"
	// configurationFile := cwd + "/output/" + prefix + "_DevicesConfigurations_" + seed + "s_" + numGateways + "x1Gv_" + numDevices + "D.dat"
	// ExportResults(s, instance, logFile, placementFile, configurationFile)
	// mainWG.Done()
}

func TSSolve(instance *problem.UAVProblem, seed, numDevices, numGateways, prefix string) {
	maxIterations := 150000
	tabuListSize := 40
	batchSize := 20
	maxIterationsWithoutEnhancement := 10000
	tabuUavRatio := float32(0.25)

	s := solver.CreateTSSolver(maxIterations, batchSize, tabuListSize, maxIterationsWithoutEnhancement, tabuUavRatio, instance)
	s.Solve()

	// ---------- Save Result

	cwd, err := os.Getwd()
	if err != nil {
		panic(err)
	}

	logFile := cwd + "/output/" + prefix + "_log_" + seed + "s_" + numGateways + "g_" + numDevices + "d.dat"
	placementFile := cwd + "/output/" + prefix + "_Placement_" + seed + "s_" + numGateways + "x1Gv_" + numDevices + "D.dat"
	configurationFile := cwd + "/output/" + prefix + "_DevicesConfigurations_" + seed + "s_" + numGateways + "x1Gv_" + numDevices + "D.dat"
	ExportResults(s, instance, logFile, placementFile, configurationFile)
}

func SASolve(instance *problem.UAVProblem) {
	initialTemp := 250.0
	coolingRate := 0.99985
	iterationsPerTemp := 20
	maxIterations := 1000000
	minDistance := 5
	maxDistance := 50

	s := solver.CreateSASolver(initialTemp, coolingRate, iterationsPerTemp, maxIterations, minDistance, maxDistance, instance)

	start := time.Now()
	s.Solve()
	end := time.Now()

	fmt.Printf("Solving time: %v\n", end.Sub(start))

	// ---------- Save Result

	//logFile := cwd + "/output/" + prefix + "_log_" + seed + "s_" + numGateways + "g_" + numDevices + "d.dat"
	//placementFile := cwd + "/output/" + prefix + "_Placement_" + seed + "s_" + numGateways + "x1Gv_" + numDevices + "D.dat"
	//configurationFile := cwd + "/output/" + prefix + "_DevicesConfigurations_" + seed + "s_" + numGateways + "x1Gv_" + numDevices + "D.dat"
	//ExportResults(s, instance, logFile, placementFile, configurationFile)
}

func ExportResults(s solver.Solver, instance *problem.UAVProblem, logFile, placementFile, configurationFile string) {
	// Log
	file, err := os.Create(logFile)
	if err != nil {
		panic(err)
	}

	_, err = file.WriteString(s.GetLog())
	if err != nil {
		panic(err)
	}

	err = file.Sync()
	if err != nil {
		panic(err)
	}
	err = file.Close()
	if err != nil {
		panic(err)
	}

	// Gateway Placement
	file, err = os.Create(placementFile)
	if err != nil {
		panic(err)
	}

	_, err = file.WriteString(instance.GetBestSolution().(*problem.UAVSolution).OutputGatewayPositions())
	if err != nil {
		panic(err)
	}

	err = file.Sync()
	if err != nil {
		panic(err)
	}
	err = file.Close()
	if err != nil {
		panic(err)
	}

	// Config Placement
	file, err = os.Create(configurationFile)
	if err != nil {
		panic(err)
	}

	_, err = file.WriteString(instance.GetBestSolution().(*problem.UAVSolution).OutputConfigurations())
	if err != nil {
		panic(err)
	}

	err = file.Sync()
	if err != nil {
		panic(err)
	}
	err = file.Close()
	if err != nil {
		panic(err)
	}
}
