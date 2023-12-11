package solver

import (
	"cmp"
	"fmt"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/device"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/problem"
	"math/rand"
	"os"
	"slices"
	"time"
)

type GRASP struct {
	problemInstance problem.Problem
	coverage        map[int32][]device.DeviceId
	uavs            []int32
	startTime       time.Time
	devicePriority  map[device.DeviceId]int
}

func CreateGRASPSolver(instance problem.Problem) *GRASP {
	return &GRASP{
		problemInstance: instance,
	}
}

func (solver *GRASP) Solve() problem.Solution {
	solver.startTime = time.Now()

	solution := solver.constructGreedyRandomizedSolution()
	solver.problemInstance.SetCurrentSolution(solution)
	solution = solver.localSearch(solution)

	return solution
}

func (solver *GRASP) localSearch(solution problem.Solution) problem.Solution {
	maxIterations := 50000
	tabuListSize := 25
	batchSize := 20
	maxIterationsWithoutEnhancement := maxIterations
	tabuUavRatio := float32(0.25)

	s := CreateTSSolver(maxIterations, batchSize, tabuListSize, maxIterationsWithoutEnhancement, tabuUavRatio, solver.problemInstance)
	return s.Solve()
}

func (solver *GRASP) constructGreedyRandomizedSolution() problem.Solution {
	solver.uavs = solver.problemInstance.GetUAVIds()
	solver.devicePriority = make(map[device.DeviceId]int, len(solver.problemInstance.GetDeviceIds()))
	solver.coverage = make(map[int32][]device.DeviceId, len(solver.problemInstance.GetUAVIds()))
	for _, uavId := range solver.problemInstance.GetUAVIds() {
		solver.coverage[uavId] = solver.problemInstance.GetCoverage(uavId)
		for _, deviceId := range solver.coverage[uavId] {
			solver.devicePriority[deviceId] += len(solver.coverage[uavId])
		}
	}

	uncoveredDevices := solver.problemInstance.GetDeviceIds()
	coverUavs := make([]int32, 0)
	mapCoverage := make(map[int32][]device.DeviceId, 1)

	for len(uncoveredDevices) > 0 {
		candidates := solver.getCandidates(0.8)

		idxChosen := rand.Intn(len(candidates))
		uavIdChosen := candidates[idxChosen]

		coverUavs = append(coverUavs, uavIdChosen)
		covered := solver.processCoveredDevices(uavIdChosen, 10, 0.9, uncoveredDevices)
		mapCoverage[uavIdChosen] = covered

		uncoveredDevices = solver.adaptGreedyFunction(covered, uncoveredDevices)

		idx := slices.Index(solver.uavs, uavIdChosen)
		solver.uavs = append(solver.uavs[:idx], solver.uavs[idx+1:]...)

	}

	solution, err := problem.GetUAVSolution(solver.problemInstance.(*problem.UAVProblem), coverUavs, mapCoverage, 10)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error: %v\n", err)
	}

	return solution
}

func (solver *GRASP) processCoveredDevices(uavId int32, defaultSF int16, alpha float32, uncoveredDevices []device.DeviceId) []device.DeviceId {
	covered := make([]device.DeviceId, 0)
	slices.SortFunc(uncoveredDevices, func(i, j device.DeviceId) int {
		return cmp.Compare(solver.devicePriority[i], solver.devicePriority[j])
	})

	usedCapacity := make(map[int32]float32, 3)
	for _, deviceId := range uncoveredDevices {
		if !slices.Contains(solver.coverage[uavId], deviceId) {
			continue
		}

		slice := solver.problemInstance.GetSlice(deviceId)
		datarate := solver.problemInstance.GetDatarate(defaultSF, slice)
		maxDatarate := solver.problemInstance.GetMaxDatarate(slice)

		if usedCapacity[slice]+datarate > maxDatarate*alpha {
			continue
		}

		usedCapacity[slice] += datarate
		covered = append(covered, deviceId)
	}

	return covered
}

func (solver *GRASP) adaptGreedyFunction(covered []device.DeviceId, uncoveredDevices []device.DeviceId) []device.DeviceId {
	for _, deviceId := range covered {
		idx := slices.Index(uncoveredDevices, deviceId)
		if idx == -1 {
			continue
		}

		uncoveredDevices = append(uncoveredDevices[:idx], uncoveredDevices[idx+1:]...)
		for _, uavId := range solver.uavs {
			idx := slices.Index(solver.coverage[uavId], deviceId)
			if idx == -1 {
				continue
			}

			solver.coverage[uavId] = append(solver.coverage[uavId][:idx], solver.coverage[uavId][idx+1:]...)
		}

	}

	return uncoveredDevices
}

func (solver *GRASP) getCandidates(alpha float64) []int32 {
	uavs := solver.uavs
	slices.SortFunc(uavs, func(i, j int32) int {
		return cmp.Compare(len(solver.coverage[j]), len(solver.coverage[i]))
	})

	maxCoverage := len(solver.coverage[uavs[0]])
	comp := int(float64(maxCoverage) * alpha)
	size := 0
	for _, uavId := range uavs {
		if len(solver.coverage[uavId]) > comp {
			size++
		} else {
			break
		}
	}
	return uavs[:size]
}
