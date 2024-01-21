package solver

import (
	"cmp"
	"fmt"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/problem"
	"math/rand"
	"slices"
)

type population struct {
	individuals []problem.Solution
	sumFitness  float64
	maxFitness  float64
	avgFitness  float64
	sumCost     float64
	minCost     float64
	avgCost     float64
}

func CreatePopulationRandom(instance problem.Problem, size int) *population {
	individuals := make([]problem.Solution, size)
	sumFitness := 0.0
	maxFitness := 0.0
	sumCost := 0.0
	minCost := 0.0

	for i := 0; i < size; i++ {
		sol, err := instance.GetRandomSolution()
		if err != nil {
			fmt.Errorf("error creating random solution: %v", err)
		}

		sumFitness += sol.GetInverseCost()
		if sol.GetInverseCost() > maxFitness {
			maxFitness = sol.GetInverseCost()
		}

		sumCost += sol.GetCost()
		if sol.GetCost() < minCost || i == 0 {
			minCost = sol.GetCost()
		}

		individuals[i] = sol
	}

	return &population{
		individuals: individuals,
		sumFitness:  sumFitness,
		maxFitness:  maxFitness,
		avgFitness:  sumFitness / float64(size),
		sumCost:     sumCost,
		minCost:     minCost,
		avgCost:     sumCost / float64(size),
	}
}

func CreatePopulationEmpty() *population {
	return &population{
		individuals: make([]problem.Solution, 0),
	}
}

func (p *population) GetIndividuals() []problem.Solution {
	return p.individuals
}

func (p *population) Size() int {
	return len(p.individuals)
}

func (p *population) SumFitness() float64 {
	return p.sumFitness
}

func (p *population) UpdateMetrics() {
	p.sumFitness = 0.0
	p.maxFitness = 0.0
	p.sumCost = 0.0
	p.minCost = 0.0

	for i := 0; i < p.Size(); i++ {
		sol := p.individuals[i]
		p.sumFitness += sol.GetInverseCost()
		if sol.GetInverseCost() > p.maxFitness {
			p.maxFitness = sol.GetInverseCost()
		}

		p.sumCost += sol.GetCost()
		if sol.GetCost() < p.minCost || i == 0 {
			p.minCost = sol.GetCost()
		}
	}

	p.avgCost = p.sumCost / float64(p.Size())
	p.avgFitness = p.sumFitness / float64(p.Size())
}

func (p *population) SelectIndividual() problem.Solution {
	sum := 0.0
	selected := -1
	r := rand.Float64() * p.SumFitness()

	for sum < r && selected < (p.Size()-1) {
		selected++
		sum += p.individuals[selected].GetInverseCost()
	}

	return p.individuals[selected]
}

func (p *population) GetBestIndividual() problem.Solution {
	p.SortPopulation()
	return p.individuals[0]
}

func (p *population) RemoveBestIndividuals() []problem.Solution {
	p.SortPopulation()
	best := p.individuals[:5]
	p.individuals = p.individuals[5:]
	return best
}

func (p *population) SortPopulation() {
	slices.SortFunc(p.individuals, func(i, j problem.Solution) int {
		return cmp.Compare(i.GetCost(), j.GetCost())
	})
}

func (p *population) AddIndividual(individual problem.Solution) {
	p.individuals = append(p.individuals, individual)
	cost := individual.GetCost()
	icost := individual.GetInverseCost()

	if len(p.individuals) == 1 {
		p.maxFitness = icost
		p.minCost = cost
	}

	p.sumFitness += icost
	p.avgFitness = p.sumFitness / float64(p.Size())
	if icost > p.maxFitness {
		p.maxFitness = icost
	}

	p.sumCost += cost
	p.avgCost = p.sumCost / float64(p.Size())
	if cost < p.minCost {
		p.minCost = cost
	}
}
