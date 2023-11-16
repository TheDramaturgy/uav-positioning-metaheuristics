package solver

import (
	"fmt"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/device"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/problem"
	"math/rand"
	"slices"
	"time"
)

type tabuMove struct {
	DeviceId  device.DeviceId
	Direction int
}

type TSSolver struct {
	problemInstance problem.Problem
	maxIterations   int
	batchSize       int
	tabuListSize    int
	tabuList        []tabuMove
	startTime       time.Time
}

func CreateTSSolver(maxIteration, batchSize, tabuListSize int, problem problem.Problem) *TSSolver {
	s := &TSSolver{
		maxIterations:   maxIteration,
		batchSize:       batchSize,
		tabuListSize:    tabuListSize,
		problemInstance: problem,
	}

	return s
}

func (solver *TSSolver) Solve() problem.Solution {
	solver.startTime = time.Now()
	solver.tabuList = make([]tabuMove, 0)
	currSolution := solver.problemInstance.GetCurrentSolution()
	solver.problemInstance.SetBestSolution(currSolution)

	for i := 0; i < solver.maxIterations; i++ {
		candidates := solver.problemInstance.GetCurrentSolution().GetNeighbourList(solver.batchSize)
		nextSolution, candidateSolutionFound, isTabuMove := solver.evaluateCandidates(candidates)

		currCost := solver.problemInstance.GetCurrentSolution().GetCost()
		bestCost := solver.problemInstance.GetBestSolution().GetCost()

		checkpoint := time.Since(solver.startTime)
		tabuListSize := len(solver.tabuList)
		if candidateSolutionFound {
			nextCost := nextSolution.GetCost()
			fmt.Printf("it: %d | currSolution: %f | nextSolution: %f | tabu: %t | bestSolution: %f | tabuListSize: %d | time: %v\n", i, currCost, nextCost, isTabuMove, bestCost, tabuListSize, checkpoint)

			if !isTabuMove {
				move := nextSolution.GetGeneratingMove()
				solver.addTabuMove(tabuMove{move.DeviceId, move.Direction})
			}
			solver.problemInstance.SetCurrentSolution(nextSolution)
			if nextCost < bestCost {
				solver.problemInstance.SetBestSolution(nextSolution)
			}
		} else {
			fmt.Printf("it: %d | currSolution: %f | NO MOVE! | bestSolution: %f | tabuListSize: %d | time: %v\n", i, currCost, bestCost, tabuListSize, checkpoint)
		}
	}

	return solver.problemInstance.GetBestSolution()
}

func (solver *TSSolver) evaluateCandidates(candidates []problem.Solution) (problem.Solution, bool, bool) {
	slices.SortFunc(candidates, func(i, j problem.Solution) int { return int(i.GetCost() - j.GetCost()) })

	currCost := solver.problemInstance.GetCurrentSolution().GetCost()
	bestCost := solver.problemInstance.GetBestSolution().GetCost()

	tabuCandidates := make([]problem.Solution, 0, solver.batchSize)
	nonTabuCandidates := make([]problem.Solution, 0, solver.batchSize)

	candidateSolutionFound := false
	isTabuMove := false
	var nextSolution problem.Solution
	bestCandidateCost := candidates[0].GetCost()

	for _, candidate := range candidates {
		if candidate.GetCost() > bestCandidateCost {
			if len(nonTabuCandidates) > 0 {
				idx := rand.Intn(len(nonTabuCandidates))
				nextSolution = nonTabuCandidates[idx]
				isTabuMove = false
			} else {
				idx := rand.Intn(len(tabuCandidates))
				nextSolution = tabuCandidates[idx]
				isTabuMove = true
				if candidate.GetCost() > bestCost {
					bestCandidateCost = candidate.GetCost()
					continue
				}
			}

			if nextSolution.GetCost() <= currCost {
				candidateSolutionFound = true
			} else {
				candidateSolutionFound = false
			}

			break
		}

		move := candidate.GetGeneratingMove()
		if slices.Contains(solver.tabuList, tabuMove{move.DeviceId, move.Direction}) {
			tabuCandidates = append(tabuCandidates, candidate)
		} else {
			nonTabuCandidates = append(nonTabuCandidates, candidate)
		}
	}

	return nextSolution, candidateSolutionFound, isTabuMove
}

func (solver *TSSolver) addTabuMove(move tabuMove) {
	if len(solver.tabuList) >= solver.tabuListSize {
		solver.tabuList = solver.tabuList[1:]
	}
	solver.tabuList = append(solver.tabuList, move)
}
