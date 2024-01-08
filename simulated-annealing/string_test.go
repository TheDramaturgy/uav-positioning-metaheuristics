package main

import (
	"fmt"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/device"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/gateway"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/problem"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/solver"
	"os"
	"strings"
	"testing"
)

func BenchmarkConcat(b *testing.B) { // 58.5
	for i := 0; i < b.N; i++ {
		var s string = ""
		for j := 0; j < 100000; j++ {
			s += fmt.Sprintf("%d,%f\n", j, (0.4 * float64(j)))
		}
	}
}

func BenchmarkBuilder(b *testing.B) { // 58.5
	for i := 0; i < b.N; i++ {
		var s strings.Builder
		s.Grow(32)
		for j := 0; j < 100000; j++ {
			s.WriteString(fmt.Sprintf("%d,%f\n", j, (0.4 * float64(j))))
		}
		_ = s.String()
	}
}

func BenchmarkBuilderWithAlloc(b *testing.B) { // 58.5
	for i := 0; i < b.N; i++ {
		var s strings.Builder
		s.Grow(32 * 100000)
		for j := 0; j < 100000; j++ {
			s.WriteString(fmt.Sprintf("%d,%f\n", j, (0.4 * float64(j))))
		}
		_ = s.String()
	}
}

//func BenchmarkTSSolve(b *testing.B) {
//	seed := "1"
//	numDevices := "50"
//	numGateways := "64"
//
//	// ---------- Files path names
//	cwd, err := os.Getwd()
//	if err != nil {
//		panic(err)
//	}
//
//	devicePositionFile := cwd + "/data/endDevices_LNM_Placement_" + seed + "s+" + numDevices + "d.dat"
//	sliceAssociationFile := cwd + "/data/skl_" + seed + "s_" + numGateways + "x1Gv_" + numDevices + "D.dat"
//	gatewayPositionFile := cwd + "/data/equidistantPlacement_" + numGateways + ".dat"
//
//	// ---------- Load Data
//	deviceList := device.ReadDeviceList(devicePositionFile, sliceAssociationFile)
//	fmt.Printf("Successfully loaded %d devices\n", deviceList.Count())
//
//	candidatePosList := gateway.ReadCandidatePositionList(gatewayPositionFile)
//	fmt.Printf("Successfully loaded %d candidate positions\n", candidatePosList.Count())
//
//	gw := &gateway.Gateway{}
//	gw.SetSensitivity(map[int16]float32{7: -130.0, 8: -132.5, 9: -135.0, 10: -137.5, 11: -140.0, 12: -142.5})
//	for slice := range deviceList.Slices() {
//		gw.AddSlice(int32(slice), 125000.0, 15197.75390625)
//	}
//
//	// ---------- Create problem instance
//	instance, err := problem.CreateUAVProblemInstance(100.0, 1.0, 0.75, 0.05, deviceList, candidatePosList, gw)
//	if err != nil {
//		panic(err)
//	}
//
//	for i := 0; i < b.N; i++ {
//		maxIterations := 1000
//		tabuListSize := 40
//		batchSize := 50
//
//		s := solver.CreateTSSolver(maxIterations, batchSize, tabuListSize, instance)
//		s.Solve()
//	}
//}

func BenchmarkGASolve(b *testing.B) {
	seed := "1"
	numDevices := "80"
	numGateways := "64"

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

	for i := 0; i < b.N; i++ {
		s := solver.CreateGASolver(instance, 500, 100, 0.6, 0.0001)
		s.Solve()
	}
}
