package main

import (
	"fmt"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/device"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/gateway"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/problem"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/solver"
	"os"
	"time"
)

func main() {

	// ---------- Parse Arguments
	args := os.Args[1:]
	seed := args[0]
	numDevices := args[1]
	numGateways := args[2]
	//prefix := args[3]

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
	instance, err := problem.CreateUAVProblemInstance(100.0, 1.0, 0.75, 0.05, deviceList, candidatePosList, gw)
	if err != nil {
		panic(err)
	}

	// ---------- Solve problem
	//SASolve(instance)
	TSSolve(instance)
}

func TSSolve(instance *problem.UAVProblem) {
	maxIterations := 100000
	tabuListSize := 40
	batchSize := 20

	s := solver.CreateTSSolver(maxIterations, batchSize, tabuListSize, instance)
	s.Solve()
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

func ExportResults(s *solver.SASolver, instance *problem.UAVProblem, logFile, placementFile, configurationFile string) {
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
