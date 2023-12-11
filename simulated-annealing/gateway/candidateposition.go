package gateway

import (
	"encoding/csv"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/utils"
	"os"
	"strconv"
	"strings"
)

type CandidatePosition struct {
	id  int32
	pos utils.Position
}

type CandidatePositionList struct {
	count        int32
	candidates   map[int32]*CandidatePosition
	candidateIds []int32
}

func (cpl *CandidatePositionList) Copy() *CandidatePositionList {
	candidatePositionList := CandidatePositionList{
		count:        cpl.count,
		candidates:   make(map[int32]*CandidatePosition, 0),
		candidateIds: make([]int32, len(cpl.candidateIds)),
	}

	for key, value := range cpl.candidates {
		candidatePositionList.candidates[key] = value
	}

	copy(candidatePositionList.candidateIds, cpl.candidateIds)

	return &candidatePositionList
}

func ReadCandidatePositionList(filePath string) *CandidatePositionList {
	candidatePositionList := CandidatePositionList{
		count:      0,
		candidates: make(map[int32]*CandidatePosition, 0),
	}

	file, err := os.Open(filePath)
	if err != nil {
		panic(err)
	}
	defer file.Close()

	reader := csv.NewReader(file)

	for {
		candidatePos, err := reader.Read()

		if candidatePos == nil {
			break
		}

		if err != nil {
			panic(err)
		}

		candidatePos = strings.Split(candidatePos[0], " ")
		candidatePosX, err := strconv.ParseFloat(candidatePos[0], 32)
		if err != nil {
			panic(err)
		}

		candidatePosY, err := strconv.ParseFloat(candidatePos[1], 32)
		if err != nil {
			panic(err)
		}

		candidatePosZ, err := strconv.ParseFloat(candidatePos[2], 32)
		if err != nil {
			panic(err)
		}
		candidate := NewCandidatePosition(float32(candidatePosX), float32(candidatePosY), float32(candidatePosZ))
		candidatePositionList.addCandidatePosition(candidate)
	}

	return &candidatePositionList
}

func NewCandidatePosition(x, y, z float32) *CandidatePosition {
	return &CandidatePosition{
		pos: utils.Position{x, y, z},
	}
}

func (candidates *CandidatePositionList) Count() int32 {
	return candidates.count
}

func (cpl *CandidatePositionList) addCandidatePosition(candidate *CandidatePosition) {
	candidate.id = cpl.count
	cpl.candidates[candidate.id] = candidate
	cpl.candidateIds = append(cpl.candidateIds, candidate.id)
	cpl.count++
}

func (candidates *CandidatePositionList) GetCandidatePosition(posId int32) utils.Position {
	return candidates.candidates[posId].pos
}

func (cpl *CandidatePositionList) GetCandidatePositionIdList() []int32 {
	ids := make([]int32, len(cpl.candidateIds))
	copy(ids, cpl.candidateIds)
	return ids
}
