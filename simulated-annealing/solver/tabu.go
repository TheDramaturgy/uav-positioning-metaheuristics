package solver

import (
	"fmt"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/device"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/problem"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/utils"
	"math/rand"
	"slices"
	"strings"
	"time"
)

const Beta float64 = 0.3

type tabuMove struct {
	DeviceId  device.DeviceId
	Direction int
}

// type associationScore struct {
// 	association problem.Association
// 	score       float64
// }

type TSSolver struct {
	problemInstance problem.Problem
	currIteration   int
	maxIterations   int
	maxIterationsWE int
	batchSize       int
	tabuListSize    int
	tabuList        []tabuMove
	tabuUav         []int32
	tabuUavRatio    float32
	eliteSolution   problem.Solution
	log             strings.Builder
	startTime       time.Time
	checkpoint      time.Duration
	bestCostTime    float64
}

func CreateTSSolver(maxIteration, batchSize, tabuListSize, maxIterationsWithoutEnhancement int, tabuUavPercentage float32, instance problem.Problem) *TSSolver {

	s := &TSSolver{
		maxIterations:   maxIteration,
		batchSize:       batchSize,
		tabuListSize:    tabuListSize,
		maxIterationsWE: maxIterationsWithoutEnhancement,
		problemInstance: instance,
		tabuUav:         make([]int32, 0),
		tabuUavRatio:    tabuUavPercentage,
	}

	s.log.Grow(64 * maxIteration)
	s.log.WriteString("it,currCost,nextCost,tabu,bestCost,timestamp\n")

	return s
}

func (solver *TSSolver) GetLog() string {
	return solver.log.String()
}

func (solver *TSSolver) GetCurrentIteration() int {
	return solver.currIteration
}

func (solver *TSSolver) GetBestCostTime() float64 {
	return solver.bestCostTime
}

func (solver *TSSolver) Solve() problem.Solution {
	solver.startTime = time.Now()
	solver.tabuList = make([]tabuMove, 0)
	currSolution := solver.problemInstance.GetCurrentSolution()
	solver.problemInstance.SetBestSolution(currSolution)

	for solver.currIteration = 0; solver.currIteration < solver.maxIterations; solver.currIteration++ {
		//fmt.Printf("\n\n==========================\n  starting intensification  \n==========================\n\n")
		solver.intensification()
		//fmt.Printf("\n\n==========================\n  end of intensification  \n==========================\n\n")

		if solver.currIteration < solver.maxIterations-1 {
			//fmt.Printf("\n\n==========================\n  starting diversification  \n==========================\n\n")
			solver.diversificationLongTermMemory()
			//fmt.Printf("\n\n==========================\n  end of diversification  \n==========================\n\n")
		}

		if solver.checkpoint.Seconds() > 60 {
			break
		}
	}

	return solver.problemInstance.GetBestSolution()
}

func (solver *TSSolver) intensification() {
	iterationsWithoutEnhancement := 0
	solver.eliteSolution = solver.problemInstance.GetCurrentSolution()

	for ; solver.currIteration < solver.maxIterations; solver.currIteration++ {
		iterationsWithoutEnhancement++
		candidates := solver.problemInstance.GetCurrentSolution().GetNeighbourList(solver.batchSize)
		nextSolution, candidateSolutionFound, isTabuMove := solver.evaluateCandidates(candidates)

		currCost := solver.problemInstance.GetCurrentSolution().GetCost()
		bestCost := solver.problemInstance.GetBestSolution().GetCost()

		solver.checkpoint = time.Since(solver.startTime)
		tabuListSize := len(solver.tabuList)

		if candidateSolutionFound {
			nextCost := nextSolution.GetCost()
			nextUavTabuRatio := nextSolution.GetUavTabuRatio(solver.tabuUav)
			fmt.Printf("it: %d | currSolution: %f | nextSolution: %f | tabu: %t | tabuUavRatio: %f | bestSolution: %f | tabuListSize: %d | time: %v\n", solver.currIteration, currCost, nextCost, isTabuMove, nextUavTabuRatio, bestCost, tabuListSize, solver.checkpoint.Seconds())
			solver.log.WriteString(fmt.Sprintf("%d,%f,%f,%t,%f,%v\n", solver.currIteration, currCost, nextCost, isTabuMove, bestCost, solver.checkpoint.Seconds()))

			move := nextSolution.GetGeneratingMove()

			if !isTabuMove {
				solver.addTabuMove(tabuMove{move.DeviceId, move.Direction})
			}

			solver.problemInstance.SetCurrentSolution(nextSolution)

			if nextCost < bestCost {
				solver.problemInstance.SetBestSolution(nextSolution)
				solver.bestCostTime = solver.checkpoint.Seconds()
			}

			if nextCost < solver.eliteSolution.GetCost() {
				iterationsWithoutEnhancement = 0
				solver.eliteSolution = nextSolution
			}

		} else {
			fmt.Printf("it: %d | currSolution: %f | NO MOVE! | bestSolution: %f | tabuListSize: %d | time: %v\n", solver.currIteration, currCost, bestCost, tabuListSize, solver.checkpoint.Seconds())
			solver.log.WriteString(fmt.Sprintf("%d,%f,%f,%t,%f,%v\n", solver.currIteration, currCost, -1.0, isTabuMove, bestCost, solver.checkpoint.Seconds()))
		}

		if iterationsWithoutEnhancement > solver.maxIterationsWE {
			break
		}

		if solver.checkpoint.Seconds() > 60 {
			break
		}
	}

}

func (solver *TSSolver) diversificationRandom() {
	exploreSolution := solver.problemInstance.GetBestSolution().GetNeighbourRandom(50, 250)
	solver.problemInstance.SetCurrentSolution(exploreSolution)
}

func (solver *TSSolver) diversificationLongTermMemory() {
	solver.tabuUav = append(solver.tabuUav, solver.eliteSolution.GetDeployedUavs()...)
	solver.tabuUav = utils.Unique(&solver.tabuUav)

	newSolution, err := problem.GetRandomUAVSolutionTabu(solver.problemInstance.(*problem.UAVProblem), solver.tabuUav, solver.tabuUavRatio)
	if err != nil {
		panic(err)
	}

	solver.problemInstance.SetCurrentSolution(newSolution)
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

	for idx, candidate := range candidates {
		if candidate.GetCost() > bestCandidateCost || idx == len(candidates)-1 {
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
		if solver.isTabuMove(move) {
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

// func (solver *TSSolver) resetEliteFreq() {
// 	for key, _ := range solver.eliteFreq {
// 		solver.eliteFreq[key] = 0
// 	}
// }

func (solver *TSSolver) isTabuMove(move problem.Move) bool {
	return slices.Contains(solver.tabuList, tabuMove{move.DeviceId, move.Direction})
}
