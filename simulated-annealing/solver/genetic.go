package solver

import (
	"fmt"
	"os"
	"strings"
	"sync"
	"time"

	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/problem"
)

type GA struct {
	problemInstance   problem.Problem
	oldPopulation     *population
	newPopulation     *population
	infeasible        int
	maxPopulation     int
	crossRate         float64
	mutationRate      float64
	maxGen            int
	iteration         int
	log               strings.Builder
	lastLog           string
	wg                *sync.WaitGroup
	mu                *sync.Mutex
	startTime         time.Time
	bestCostTime      float64
	maxIterationsTabu int
}

func CreateGASolver(instance problem.Problem, maxGen, populationSize, maxTabuIterations int, crossRate, mutationRate float64) *GA {
	population := CreatePopulationRandom(instance, populationSize)

	// population := CreatePopulationEmpty()
	// for i := 0; i < populationSize; i++ {
	// 	s := CreateGRASPSolver(instance.Copy())
	// 	population.AddIndividual(s.SolveFast())
	// }

	return &GA{
		problemInstance:   instance,
		newPopulation:     population,
		maxGen:            maxGen,
		maxIterationsTabu: maxTabuIterations,
		maxPopulation:     populationSize,
		crossRate:         crossRate,
		mutationRate:      mutationRate,
		wg:                &sync.WaitGroup{},
		mu:                &sync.Mutex{},
	}
}

func (solver *GA) GetLog() string {
	return solver.log.String()
}

func (solver *GA) GetLastLog() string {
	return solver.lastLog
}

func (solver *GA) GetBestCostTime() float64 {
	return solver.bestCostTime
}

func (solver *GA) GetCurrentIteration() int {
	return solver.iteration
}

func (solver *GA) Solve() problem.Solution {
	fmt.Printf("Solving with GA\n")
	solver.startTime = time.Now()

	solver.problemInstance.SetBestSolution(solver.newPopulation.GetBestIndividual())

	solver.log.WriteString("gen,bestCost,avgCost,infeasible,population,time\n")
	solver.log.WriteString(fmt.Sprintf("%d, %f, %f, %d, %d, 0.0\n", 0, solver.newPopulation.minCost, solver.newPopulation.avgCost, solver.infeasible, solver.newPopulation.Size()))
	solver.lastLog = fmt.Sprintf("it: 0 | bestCost: %f | avgCost: %f | infe: %d | Population: %d | time: 0.0\n", solver.newPopulation.minCost, solver.newPopulation.avgCost, solver.infeasible, solver.newPopulation.Size())
	fmt.Printf(solver.lastLog)

	for solver.iteration = 1; solver.iteration <= solver.maxGen; solver.iteration++ {
		solver.reproduce()
		checkpoint := time.Since(solver.startTime)
		solver.log.WriteString(fmt.Sprintf("%d, %f, %f, %d, %d, %v\n", solver.iteration, solver.newPopulation.minCost, solver.newPopulation.avgCost, solver.infeasible, solver.newPopulation.Size(), checkpoint.Seconds()))
		solver.lastLog = fmt.Sprintf("it: %d | bestCost: %f | avgCost: %f | infe: %d | Population: %d | time: %v\n", solver.iteration, solver.newPopulation.minCost, solver.newPopulation.avgCost, solver.infeasible, solver.newPopulation.Size(), checkpoint.Seconds())
		fmt.Printf(solver.lastLog)

		if solver.problemInstance.GetBestSolution().GetCost() > solver.newPopulation.GetBestIndividual().GetCost() {
			solver.problemInstance.SetBestSolution(solver.newPopulation.GetBestIndividual())
			solver.bestCostTime = checkpoint.Seconds()
		}

		if checkpoint.Seconds() > 60 {
			break
		}
	}

	return solver.problemInstance.GetBestSolution()
}

func (solver *GA) Test() {
	// Gateway Placement
	cwd, err := os.Getwd()
	if err != nil {
		panic(err)
	}

	placementFile := cwd + "/output/test_p1_Placement_1s_64x1Gv_80D.csv"
	file, err := os.Create(placementFile)
	if err != nil {
		panic(err)
	}

	_, err = file.WriteString(solver.newPopulation.individuals[0].(*problem.UAVSolution).OutputGatewayPositions())
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
	configurationFile := cwd + "/output/test_p1_Config_1s_64x1Gv_80D.csv"
	file, err = os.Create(configurationFile)
	if err != nil {
		panic(err)
	}

	_, err = file.WriteString(solver.newPopulation.individuals[0].(*problem.UAVSolution).OutputConfigurations())
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

	placementFile = cwd + "/output/test_p2_Placement_1s_64x1Gv_80D.csv"
	file, err = os.Create(placementFile)
	if err != nil {
		panic(err)
	}

	_, err = file.WriteString(solver.newPopulation.individuals[1].(*problem.UAVSolution).OutputGatewayPositions())
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
	configurationFile = cwd + "/output/test_p2_Config_1s_64x1Gv_80D.csv"
	file, err = os.Create(configurationFile)
	if err != nil {
		panic(err)
	}

	_, err = file.WriteString(solver.newPopulation.individuals[1].(*problem.UAVSolution).OutputConfigurations())
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

	solver.reproduce()

	placementFile = cwd + "/output/test_c1_Placement_1s_64x1Gv_80D.csv"
	file, err = os.Create(placementFile)
	if err != nil {
		panic(err)
	}

	_, err = file.WriteString(solver.newPopulation.individuals[0].(*problem.UAVSolution).OutputGatewayPositions())
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
	configurationFile = cwd + "/output/test_c1_Config_1s_64x1Gv_80D.csv"
	file, err = os.Create(configurationFile)
	if err != nil {
		panic(err)
	}

	_, err = file.WriteString(solver.newPopulation.individuals[0].(*problem.UAVSolution).OutputConfigurations())
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

	placementFile = cwd + "/output/test_c2_Placement_1s_64x1Gv_80D.csv"
	file, err = os.Create(placementFile)
	if err != nil {
		panic(err)
	}

	_, err = file.WriteString(solver.newPopulation.individuals[1].(*problem.UAVSolution).OutputGatewayPositions())
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
	configurationFile = cwd + "/output/test_c2_Config_1s_64x1Gv_80D.csv"
	file, err = os.Create(configurationFile)
	if err != nil {
		panic(err)
	}

	_, err = file.WriteString(solver.newPopulation.individuals[1].(*problem.UAVSolution).OutputConfigurations())
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

func (solver *GA) reproduce() {
	solver.oldPopulation = solver.newPopulation
	solver.newPopulation = CreatePopulationEmpty()

	workers := 8

	out := make(chan problem.Solution, solver.maxPopulation)
	for i := 0; i < workers; i++ {
		solver.wg.Add(1)
		childs := solver.maxPopulation / workers
		if i == workers-1 {
			childs += solver.maxPopulation % workers
		}
		go solver.reproduceService(childs, out)
	}
	solver.wg.Wait()

	bestIndividuals := solver.newPopulation.RemoveBestIndividuals()
	for idx, _ := range bestIndividuals {
		individual := bestIndividuals[idx]

		maxIterations := solver.maxIterationsTabu
		tabuListSize := 25
		batchSize := 20
		maxIterationsWithoutEnhancement := maxIterations
		tabuUavRatio := float32(0.25)

		tabuSolver := CreateTSSolver(maxIterations, batchSize, tabuListSize, maxIterationsWithoutEnhancement, tabuUavRatio, solver.problemInstance.Copy())
		tabuSolver.problemInstance.SetCurrentSolution(individual)
		sol := tabuSolver.Solve()

		solver.newPopulation.AddIndividual(sol)
	}

	solver.newPopulation.UpdateMetrics()
}

func (solver *GA) reproduceService(numChilds int, out chan problem.Solution) {
	defer solver.wg.Done()
	solver.infeasible = 0
	for i := 0; i < numChilds; i += 2 {
		p1 := solver.oldPopulation.SelectIndividual()
		p2 := solver.oldPopulation.SelectIndividual()

		c1, c2 := problem.Crossover(p1.(*problem.UAVSolution), p2.(*problem.UAVSolution), solver.crossRate, solver.mutationRate)

		c1.GetCost()
		c2.GetCost()

		if !c1.IsFeasible() {
			c1 = p1.Copy().(*problem.UAVSolution)
			solver.infeasible++
		}

		if !c2.IsFeasible() {
			c2 = p2.Copy().(*problem.UAVSolution)
			solver.infeasible++
		}

		solver.mu.Lock()
		solver.newPopulation.AddIndividual(c1)
		solver.newPopulation.AddIndividual(c2)
		solver.mu.Unlock()
	}
}
