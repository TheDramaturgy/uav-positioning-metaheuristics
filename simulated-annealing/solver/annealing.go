package solver

import (
	"fmt"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/utils"
	"math"
	"strings"
	"time"
)

type Problem interface {
	GetCurrentSolution() Solution
	SetCurrentSolution(Solution)
	GetBestSolution() Solution
	SetBestSolution(Solution)
}

type Solution interface {
	GetCost() float64
	GetCostA() float64
	GetCostB() float64
	GetNeighbour(minDistance, maxDistance int) Solution
}

type Solver struct {
	problemInstance   Problem
	coolingRate       float64
	initialTemp       float64
	iterationsPerTemp int
	maxIterations     int
	minDistance       int
	maxDistance       int
	log               strings.Builder
	startTime         time.Time
}

func CreateSolver(initialTemp, coolingRate float64, iterationsPerTemp, maxIterations, minDistance, maxDistance int, problem Problem) *Solver {
	s := &Solver{
		problemInstance:   problem,
		initialTemp:       initialTemp,
		coolingRate:       coolingRate,
		iterationsPerTemp: iterationsPerTemp,
		maxIterations:     maxIterations,
		minDistance:       minDistance,
		maxDistance:       maxDistance,
	}

	s.log.Grow(64 * maxIterations)
	s.log.WriteString("it,temp,d,currCost,nextCost,bestCost,timestamp\n")
	return s
}

func (solver *Solver) GetLog() string {
	return solver.log.String()
}

func (solver *Solver) Solve() Solution {
	temp := solver.initialTemp

	currSolution := solver.problemInstance.GetCurrentSolution()
	solver.problemInstance.SetBestSolution(currSolution)

	solver.startTime = time.Now()
	numIterations := 0
	for numIterations < solver.maxIterations {
		solver.iterateOverTemp(temp, numIterations)
		numIterations += solver.iterationsPerTemp
		temp = solver.cool(temp)
	}

	return solver.problemInstance.GetBestSolution()
}

func (solver *Solver) iterateOverTemp(temp float64, it int) {
	for i := 0; i < solver.iterationsPerTemp; i++ {
		currSolution := solver.problemInstance.GetCurrentSolution()
		nextSolution := currSolution.GetNeighbour(1, 5)

		currCost := currSolution.GetCost()
		nextCost := nextSolution.GetCost()
		bestCost := solver.problemInstance.GetBestSolution().GetCost()

		d := math.Exp(-(nextCost - currCost) / temp)
		if nextCost <= currCost {
			d = -1
		}

		checkpoint := time.Since(solver.startTime)
		fmt.Printf("it: %d | temp: %f | d: %f | currSolution: %f | nextSolution: %f | bestSolution: %f | time: %v\n", it+i, temp, d, currCost, nextCost, bestCost, checkpoint)
		solver.log.WriteString(fmt.Sprintf("%d,%f,%f,%f,%f,%f,%v\n", it+i, temp, d, currCost, nextCost, bestCost, checkpoint))

		if nextCost <= currCost {
			solver.problemInstance.SetCurrentSolution(nextSolution)
		} else {
			if utils.GetRandomProbability() < d {
				solver.problemInstance.SetCurrentSolution(nextSolution)
			}
		}

		if nextCost <= bestCost {
			solver.problemInstance.SetBestSolution(nextSolution)
		}

	}
}

func (solver *Solver) cool(temp float64) float64 {
	return temp * solver.coolingRate
}
