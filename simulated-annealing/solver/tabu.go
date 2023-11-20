package solver

import (
	"fmt"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/device"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/problem"
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
	maxEliteSol     int
	currEliteSol    problem.Solution
	eliteSol        []problem.Solution
	eliteFreq       map[problem.Association]int
	flipFreq        map[problem.Association]int
	maxFreq         int
	log             strings.Builder
	startTime       time.Time
}

func CreateTSSolver(maxIteration, batchSize, tabuListSize, maxIterationsWithoutEnhancement int, instance problem.Problem) *TSSolver {
	eliteFreq := make(map[problem.Association]int)
	flipFreq := make(map[problem.Association]int)

	for _, deviceId := range instance.GetDeviceIds() {
		for _, uavId := range instance.GetPossibleUavs(deviceId) {
			for _, configId := range instance.GetPossibleConfigs(deviceId, uavId) {
				key := problem.Association{Device: deviceId, Uav: uavId, Config: configId}
				eliteFreq[key] = 0
				flipFreq[key] = 0
			}
		}
	}

	s := &TSSolver{
		maxIterations:   maxIteration,
		batchSize:       batchSize,
		tabuListSize:    tabuListSize,
		maxIterationsWE: maxIterationsWithoutEnhancement,
		problemInstance: instance,
		maxEliteSol:     8,
		eliteFreq:       eliteFreq,
		flipFreq:        flipFreq,
		maxFreq:         0,
	}

	s.log.Grow(64 * maxIteration)
	s.log.WriteString("it,currCost,nextCost,tabu,bestCost,timestamp\n")

	return s
}

func (solver *TSSolver) GetLog() string {
	return solver.log.String()
}

func (solver *TSSolver) Solve() problem.Solution {
	solver.startTime = time.Now()
	solver.tabuList = make([]tabuMove, 0)
	currSolution := solver.problemInstance.GetCurrentSolution()
	solver.problemInstance.SetBestSolution(currSolution)

	for solver.currIteration = 0; solver.currIteration < solver.maxIterations; solver.currIteration++ {
		fmt.Printf("\n\n==========================\n  starting intensification  \n==========================\n\n")
		solver.intensification()
		solver.storeCurrEliteSol()
		fmt.Printf("\n\n==========================\n  end of intensification  \n==========================\n\n")

		if solver.currIteration < solver.maxIterations-1 {
			fmt.Printf("\n\n==========================\n  starting diversification  \n==========================\n\n")
			solver.diversificationRandom()
			fmt.Printf("\n\n==========================\n  end of diversification  \n==========================\n\n")
		}
	}

	return solver.problemInstance.GetBestSolution()
}

func (solver *TSSolver) intensification() {
	iterationsWithoutEnhancement := 0
	solver.currEliteSol = solver.problemInstance.GetCurrentSolution()

	for ; solver.currIteration < solver.maxIterations; solver.currIteration++ {
		iterationsWithoutEnhancement++
		candidates := solver.problemInstance.GetCurrentSolution().GetNeighbourList(solver.batchSize)
		nextSolution, candidateSolutionFound, isTabuMove := solver.evaluateCandidates(candidates)

		currCost := solver.problemInstance.GetCurrentSolution().GetCost()
		bestCost := solver.problemInstance.GetBestSolution().GetCost()
		currEliteCost := solver.currEliteSol.GetCost()

		checkpoint := time.Since(solver.startTime)
		tabuListSize := len(solver.tabuList)
		if candidateSolutionFound {
			nextCost := nextSolution.GetCost()
			fmt.Printf("it: %d | currSolution: %f | nextSolution: %f | tabu: %t | bestSolution: %f | tabuListSize: %d | time: %v\n", solver.currIteration, currCost, nextCost, isTabuMove, bestCost, tabuListSize, checkpoint.Seconds())
			solver.log.WriteString(fmt.Sprintf("%d,%f,%f,%t,%f,%v\n", solver.currIteration, currCost, nextCost, isTabuMove, bestCost, checkpoint.Seconds()))

			move := nextSolution.GetGeneratingMove()
			solver.updateFlipFreq(move)

			if !isTabuMove {
				solver.addTabuMove(tabuMove{move.DeviceId, move.Direction})
			}

			solver.problemInstance.SetCurrentSolution(nextSolution)

			if nextCost < bestCost {
				iterationsWithoutEnhancement = 0
				solver.problemInstance.SetBestSolution(nextSolution)
			}

			if nextCost < currEliteCost {
				solver.currEliteSol = nextSolution
			}
		} else {
			fmt.Printf("it: %d | currSolution: %f | NO MOVE! | bestSolution: %f | tabuListSize: %d | time: %v\n", solver.currIteration, currCost, bestCost, tabuListSize, checkpoint)
			solver.log.WriteString(fmt.Sprintf("%d,%f,%f,%t,%f,%v\n", solver.currIteration, currCost, -1.0, isTabuMove, bestCost, checkpoint.Seconds()))
		}

		if iterationsWithoutEnhancement > solver.maxIterationsWE {
			break
		}
	}
}

func (solver *TSSolver) diversificationRandom() {
	exploreSolution := solver.problemInstance.GetBestSolution().GetNeighbourRandom(50, 250)
	solver.problemInstance.SetCurrentSolution(exploreSolution)
}

// func (solver *TSSolver) diversificationLongTermMemory() {
// 	solver.resetEliteFreq()

// 	for _, eliteSol := range solver.eliteSol {
// 		for _, as := range eliteSol.GetAssociations() {
// 			solver.eliteFreq[as]++
// 		}
// 	}

// 	scoreAs := make([]associationScore, 0, len(solver.flipFreq))
// 	for as, freq := range solver.flipFreq {
// 		r := float64(len(solver.eliteSol))
// 		score := float64(solver.eliteFreq[as]) * (r - float64(solver.eliteFreq[as])) / r * r
// 		score += Beta * (1.0 - float64(freq)/float64(solver.maxFreq))

// 		scoreAs = append(scoreAs, associationScore{as, score})
// 	}

// 	lambda := 1.001
// 	sum := 1.0
// 	//for i, _ := range scoreAs {
// 	//	sum += math.Pow(float64(i+1), -lambda)
// 	//}

// 	slices.SortFunc(scoreAs, func(i, j associationScore) int { return int(j.score - i.score) })
// 	idx := rand.Intn(len(solver.eliteSol))
// 	flipCount := 0
// 	eliteSol := solver.eliteSol[idx].Copy()
// 	for j, as := range scoreAs {
// 		if j > len(scoreAs)/4 {
// 			break
// 		}

// 		chance := rand.Float32()
// 		p := float32(math.Pow(float64(j+1), -lambda) / sum)
// 		if chance <= p {
// 			eliteSol.FlipAssociation(as.association)
// 			flipCount++
// 		}

// 	}

// 	solver.problemInstance.SetCurrentSolution(eliteSol)
// }

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

func (solver *TSSolver) updateFlipFreq(move problem.Move) {
	key := problem.Association{Device: move.DeviceId, Uav: move.PrevUAV, Config: move.PrevConfig}
	solver.flipFreq[key]++
	if solver.flipFreq[key] > solver.maxFreq {
		solver.maxFreq = solver.flipFreq[key]
	}

	key = problem.Association{Device: move.DeviceId, Uav: move.NewUAV, Config: move.NewConfig}
	solver.flipFreq[key]++
	if solver.flipFreq[key] > solver.maxFreq {
		solver.maxFreq = solver.flipFreq[key]
	}
}

// func (solver *TSSolver) resetEliteFreq() {
// 	for key, _ := range solver.eliteFreq {
// 		solver.eliteFreq[key] = 0
// 	}
// }

func (solver *TSSolver) isTabuMove(move problem.Move) bool {
	return slices.Contains(solver.tabuList, tabuMove{move.DeviceId, move.Direction})
}

func (solver *TSSolver) storeCurrEliteSol() {
	if len(solver.eliteSol) >= solver.maxEliteSol {
		slices.SortFunc(solver.eliteSol, func(i, j problem.Solution) int { return int(i.GetCost() - j.GetCost()) })
		solver.eliteSol = solver.eliteSol[:solver.maxEliteSol-1]
	}
	solver.eliteSol = append(solver.eliteSol, solver.currEliteSol)
}
