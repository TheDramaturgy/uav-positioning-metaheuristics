package problem

import (
	"errors"
	"fmt"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/device"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/gateway"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/utils"
	"math"
	"math/rand"
	"slices"
)

const (
	// Path Loss Parameters
	ReferencePrx        float32 = 10.0
	ReferenceDistance   float32 = 1.0
	AttenuationExponent float32 = 3.76

	// Quality of Service Parameters
	CondingRate float32 = 4.0 / 5.0
	PacketSize  float32 = 400
	MaxDatarate float32 = 6835.94
	MinDatarate float32 = 183.11
	MaxDelay    float32 = PacketSize / MinDatarate
	QoSBound    float32 = 0.9
)

type Problem interface {
	GetCurrentSolution() Solution
	SetCurrentSolution(Solution)
	GetBestSolution() Solution
	SetBestSolution(Solution)
	GetDeviceIds() []device.DeviceId
	GetUAVIds() []int32
	GetPossibleUavs(deviceId device.DeviceId) []int32
	GetPossibleConfigs(deviceId device.DeviceId, uavId int32) []int32
	GetCoverage(int32) []device.DeviceId
	GetDatarate(sf int16, slice int32) float32
	GetMaxDatarate(slice int32) float32
	GetSlice(deviceId device.DeviceId) int32
}

type deviceGatewayAssociation struct {
	deviceId device.DeviceId
	uavId    int32
}

type uavConfigurationAssociation struct {
	uavId    int32
	configId int32
}

type UAVProblem struct {
	gateway                *gateway.Gateway
	devices                *device.DeviceList
	uavPositions           *gateway.CandidatePositionList
	configurations         map[int32]*device.Configuration
	possibleConfigurations map[deviceGatewayAssociation][]int32
	possibleUavs           map[device.DeviceId][]int32
	coverageMap            map[int32][]device.DeviceId
	alpha                  float64
	beta                   float64
	changeUav              float64
	newUavChance           float64
	currentSolution        *UAVSolution
	bestSolution           *UAVSolution
}

func (problem *UAVProblem) GetDeviceIds() []device.DeviceId {
	return problem.devices.GetDeviceIds()
}

func (problem *UAVProblem) GetUAVIds() []int32 {
	return problem.uavPositions.GetCandidatePositionIdList()
}

func (problem *UAVProblem) GetPossibleUavs(deviceId device.DeviceId) []int32 {
	return problem.possibleUavs[deviceId]
}

func (problem *UAVProblem) GetPossibleConfigs(deviceId device.DeviceId, uavId int32) []int32 {
	return problem.possibleConfigurations[deviceGatewayAssociation{deviceId, uavId}]
}

func (problem *UAVProblem) GetCoverage(uavId int32) []device.DeviceId {
	return problem.coverageMap[uavId]
}

func (problem *UAVProblem) GetDatarate(sf int16, slice int32) float32 {
	return problem.gateway.GetDatarate(sf, slice)
}

func (problem *UAVProblem) GetMaxDatarate(slice int32) float32 {
	return problem.gateway.GetMaxDatarate(slice)
}

func (problem *UAVProblem) GetSlice(deviceId device.DeviceId) int32 {
	return problem.devices.GetDevice(deviceId).Slice()
}

func (problem *UAVProblem) checkReachFeasibility(deviceId device.DeviceId, uavId, configId int32) bool {
	tp := float32(problem.configurations[configId].Tp)
	sf := problem.configurations[configId].Sf

	devicePos := problem.devices.GetDevice(deviceId).GetPosition()
	candidatePos := problem.uavPositions.GetCandidatePosition(uavId)
	distance := candidatePos.DistanceFrom(devicePos) / ReferenceDistance

	pathloss := float32(0.0)
	if distance > ReferenceDistance {
		pathloss = 10.0 * AttenuationExponent * float32(math.Log10(float64(distance/ReferenceDistance)))
	}

	if tp-ReferencePrx-pathloss >= problem.gateway.GetSensitivityForSf(sf) {
		return true
	} else {
		return false
	}
}

func (problem *UAVProblem) checkQoSFeasibility(deviceId device.DeviceId, configId int32) bool {
	return problem.GetQoS(deviceId, configId) > QoSBound
}

func (problem *UAVProblem) GetQoS(deviceId device.DeviceId, configId int32) float32 {
	sf := problem.configurations[configId].Sf
	slice := problem.devices.GetDevice(deviceId).Slice()
	bandwidth := problem.gateway.GetBandwidth(slice)

	datarate := bandwidth * float32(sf) / float32(math.Pow(2, float64(sf))) * CondingRate
	delay := PacketSize / datarate

	return datarate/MaxDatarate + (1 - delay/MaxDelay)
}

func (problem *UAVProblem) ConstructInitialSolution() error {
	if problem.devices == nil {
		return errors.New("UAVProblem not initialized")
	}

	var err error
	problem.currentSolution, err = GetRandomUAVSolution(problem)
	if err != nil {
		panic(err)
	}

	return nil
}

func (problem *UAVProblem) GetCurrentSolution() Solution {
	if problem.currentSolution == nil {
		err := problem.ConstructInitialSolution()
		if err != nil {
			panic(err)
		}
	}

	return problem.currentSolution
}

func (problem *UAVProblem) GetChanceOfChangingUAV() float64 {
	return problem.changeUav
}

func (problem *UAVProblem) SetBestSolution(sol Solution) {
	problem.bestSolution = sol.(*UAVSolution)
}

func (problem *UAVProblem) SetCurrentSolution(sol Solution) {
	problem.currentSolution = sol.(*UAVSolution)
}

func (problem *UAVProblem) GetBestSolution() Solution {
	return problem.bestSolution
}

func (problem *UAVProblem) processPossibleConfigurationPerDevice() error {
	numDevices := problem.devices.Count()
	numCombinations := numDevices * problem.uavPositions.Count()
	problem.possibleConfigurations = make(map[deviceGatewayAssociation][]int32, numCombinations)
	problem.possibleUavs = make(map[device.DeviceId][]int32, numDevices)
	problem.coverageMap = make(map[int32][]device.DeviceId, 1)

	// for each device/candidate position association, check which configurations are feasible
	for deviceId := device.DeviceId(0); deviceId < device.DeviceId(problem.devices.Count()); deviceId++ {
		isFeasible := false
		for uavId := int32(0); uavId < problem.uavPositions.Count(); uavId++ {
			configs := make([]int32, 0)
			for configId := range problem.configurations {
				// verify whether the configuration satisfy the qos bound for the device
				if !problem.checkQoSFeasibility(deviceId, configId) {
					continue
				}

				// verify whether the device can reach the candidate position given configuration
				if !problem.checkReachFeasibility(deviceId, uavId, configId) {
					continue
				}

				// feasible association
				configs = append(configs, configId)
			}

			// if at least one configuration was found for the association, the problem is feasible
			// for this device
			if len(configs) > 0 {
				isFeasible = true
				slices.Sort(configs)
				problem.possibleConfigurations[deviceGatewayAssociation{deviceId, uavId}] = configs
				problem.possibleUavs[deviceId] = append(problem.possibleUavs[deviceId], uavId)
				problem.coverageMap[uavId] = append(problem.coverageMap[uavId], deviceId)
			}
		}

		if !isFeasible {
			return fmt.Errorf("unfeasibility: none candidate position is able to reach device %d", deviceId)
		}
	}

	// !!!! DEBUG: print possible configurations !!!!

	// for key, value := range problem.possibleConfigurations {
	// 	fmt.Printf("%+v: %+v\n", key, value)
	// }

	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	return nil
}

func (problem *UAVProblem) getRandomUavConfiguration(deviceId device.DeviceId) uavConfigurationAssociation {
	uavRandIdx := rand.Int31n(int32(len(problem.possibleUavs[deviceId])))
	uavRandId := problem.possibleUavs[deviceId][uavRandIdx]

	association := deviceGatewayAssociation{deviceId, uavRandId}
	numPossibleConfigs := len(problem.possibleConfigurations[association])

	configRandIdx := rand.Int31n(int32(numPossibleConfigs))
	configRandId := problem.possibleConfigurations[association][configRandIdx]

	return uavConfigurationAssociation{uavRandId, configRandId}
}

func (problem *UAVProblem) getConfigurationForUAV(deviceId device.DeviceId, uavId int32, SF int16) uavConfigurationAssociation {
	association := deviceGatewayAssociation{deviceId, uavId}
	numPossibleConfigs := len(problem.possibleConfigurations[association])

	for _, configId := range problem.possibleConfigurations[association] {
		if problem.configurations[configId].Sf == SF {
			return uavConfigurationAssociation{uavId, configId}
		}
	}

	configRandIdx := rand.Int31n(int32(numPossibleConfigs))
	configRandId := problem.possibleConfigurations[association][configRandIdx]

	return uavConfigurationAssociation{uavId, configRandId}
}

func (problem *UAVProblem) getRandomUavConfigurationTabu(deviceId device.DeviceId, used, tabu []int32, tabuRatio, currTabuRatio float32) uavConfigurationAssociation {
	possibleUavs := problem.possibleUavs[deviceId]

	if currTabuRatio >= tabuRatio {
		usedTabuUavs := utils.Intersection(tabu, used)
		possibleUsedTabuUavs := utils.Intersection(possibleUavs, usedTabuUavs)

		nonTabuUavs := utils.Complement(tabu, problem.uavPositions.GetCandidatePositionIdList())
		possibleUavs = utils.Intersection(possibleUavs, nonTabuUavs)
		possibleUavs = append(possibleUavs, possibleUsedTabuUavs...)
	}

	uavRandIdx := rand.Int31n(int32(len(possibleUavs)))
	uavRandId := possibleUavs[uavRandIdx]

	association := deviceGatewayAssociation{deviceId, uavRandId}
	numPossibleConfigs := len(problem.possibleConfigurations[association])

	configRandIdx := rand.Int31n(int32(numPossibleConfigs))
	configRandId := problem.possibleConfigurations[association][configRandIdx]

	return uavConfigurationAssociation{uavRandId, configRandId}
}

func (problem *UAVProblem) neighbourUav(deviceId device.DeviceId, uavId, configId int32, possibleUavs *[]int32, next bool) uavConfigurationAssociation {
	// Find current UAV index inside possible UAVs
	// fmt.Printf("SEARCH: %d\n", uavId)
	slices.Sort(*possibleUavs)
	uavIdx, found := slices.BinarySearch(*possibleUavs, uavId)
	if !found {
		panic("uavId not found")
	}

	// Get next possible UAV
	numUavs := len(*possibleUavs)
	var uavNewIdx int
	if next {
		uavNewIdx = int(math.Mod(float64(uavIdx+1), float64(numUavs)))
	} else {
		uavNewIdx = uavIdx - 1
		if uavNewIdx < 0 {
			uavNewIdx = numUavs - 1
		}
	}
	uavNewId := (*possibleUavs)[uavNewIdx]

	// Adjust Configuration if needed
	devGwAss := deviceGatewayAssociation{deviceId, uavNewId}
	_, found = slices.BinarySearch(problem.possibleConfigurations[devGwAss], configId)

	if found {
		return uavConfigurationAssociation{uavNewId, configId}
	} else {
		return problem.nextConfig(deviceId, uavNewId, configId)
	}
}

func (problem *UAVProblem) neighbourUavTabu(deviceId device.DeviceId, uavId, configId int32, possibleUavs *[]int32, tabu []int32, next bool) uavConfigurationAssociation {
	// Find current UAV index inside possible UAVs
	// fmt.Printf("SEARCH: %d\n", uavId)
	slices.Sort(tabu)
	slices.Sort(*possibleUavs)
	uavIdx, found := slices.BinarySearch(*possibleUavs, uavId)
	if !found {
		panic("uavId not found")
	}

	// Get next possible UAV
	numUavs := len(*possibleUavs)

	// Try to find non-tabu uav
	uavNewIdx := -1
	if next {
		for i := 0; i < numUavs; i++ {
			candidate := int(math.Mod(float64(uavIdx+1), float64(numUavs)))

			_, found := slices.BinarySearch(tabu, (*possibleUavs)[candidate])
			if found {
				// Tabu UAV
				continue
			} else {
				// Non-tabu UAV
				uavNewIdx = candidate
				break
			}
		}
	} else {
		for i := 0; i < numUavs; i++ {
			candidate := uavIdx - 1
			if candidate < 0 {
				candidate = numUavs - 1
			}

			_, found := slices.BinarySearch(tabu, (*possibleUavs)[candidate])
			if found {
				// Tabu UAV
				continue
			} else {
				// Non-tabu UAV
				uavNewIdx = candidate
				break
			}
		}
	}

	if uavNewIdx == -1 {
		// no non-tabu UAV found, use tabu UAV
		return problem.neighbourUav(deviceId, uavId, configId, possibleUavs, next)
	}

	uavNewId := (*possibleUavs)[uavNewIdx]

	// Adjust Configuration if needed
	devGwAss := deviceGatewayAssociation{deviceId, uavNewId}
	_, found = slices.BinarySearch(problem.possibleConfigurations[devGwAss], configId)

	if found {
		return uavConfigurationAssociation{uavNewId, configId}
	} else {
		return problem.nextConfig(deviceId, uavNewId, configId)
	}
}

func (problem *UAVProblem) nextUav(deviceId device.DeviceId, uavId, configId int32) uavConfigurationAssociation {
	possibleUavs := make([]int32, len(problem.possibleUavs[deviceId]))
	copy(possibleUavs, problem.possibleUavs[deviceId])
	return problem.neighbourUav(deviceId, uavId, configId, &possibleUavs, true)
}

func (problem *UAVProblem) previousUav(deviceId device.DeviceId, uavId, configId int32) uavConfigurationAssociation {
	possibleUavs := make([]int32, len(problem.possibleUavs[deviceId]))
	copy(possibleUavs, problem.possibleUavs[deviceId])
	return problem.neighbourUav(deviceId, uavId, configId, &possibleUavs, false)
}

func (problem *UAVProblem) nextDeployedUav(deviceId device.DeviceId, uavId, configId int32, sol *UAVSolution, tabu []int32) uavConfigurationAssociation {
	// Possible UAVs are the intersection between those available for the device and the deployed ones
	possibleUavs := utils.Intersection(problem.possibleUavs[deviceId], sol.deployedUavs)

	if len(tabu) > 0 {
		return problem.neighbourUavTabu(deviceId, uavId, configId, &possibleUavs, tabu, true)
	}
	return problem.neighbourUav(deviceId, uavId, configId, &possibleUavs, true)
}

func (problem *UAVProblem) previousDeployedUav(deviceId device.DeviceId, uavId, configId int32, sol *UAVSolution, tabu []int32) uavConfigurationAssociation {
	// Possible UAVs are the intersection between those available for the device and the deployed ones
	possibleUavs := utils.Intersection(problem.possibleUavs[deviceId], sol.deployedUavs)

	if len(tabu) > 0 {
		return problem.neighbourUavTabu(deviceId, uavId, configId, &possibleUavs, tabu, false)
	}
	return problem.neighbourUav(deviceId, uavId, configId, &possibleUavs, false)
}

func (problem *UAVProblem) nextNewUav(deviceId device.DeviceId, uavId, configId int32, sol *UAVSolution, tabu []int32, percentage float32) uavConfigurationAssociation {
	// Possible UAVs are the intersection between those available for the device and the deployed ones
	complement := []int32{uavId}
	complement = append(complement, utils.Complement(sol.deployedUavs, problem.uavPositions.GetCandidatePositionIdList())...)
	possibleUavs := utils.Intersection(problem.possibleUavs[deviceId], complement)

	if len(tabu) > 0 {
		deployedTabuUAVS := utils.Intersection(sol.deployedUavs, tabu)
		tabuUavRatio := float32(len(deployedTabuUAVS)) / float32(len(sol.deployedUavs))

		if tabuUavRatio > percentage {
			return problem.neighbourUavTabu(deviceId, uavId, configId, &possibleUavs, tabu, true)
		}
	}
	return problem.neighbourUav(deviceId, uavId, configId, &possibleUavs, true)
}

func (problem *UAVProblem) previousNewUav(deviceId device.DeviceId, uavId, configId int32, sol *UAVSolution, tabu []int32, percentage float32) uavConfigurationAssociation {
	// Possible UAVs are the intersection between those available for the device and the deployed ones
	complement := []int32{uavId}
	complement = append(complement, utils.Complement(sol.deployedUavs, problem.uavPositions.GetCandidatePositionIdList())...)
	possibleUavs := utils.Intersection(problem.possibleUavs[deviceId], complement)

	if len(tabu) > 0 {
		deployedTabuUAVS := utils.Intersection(sol.deployedUavs, tabu)
		tabuUavRatio := float32(len(deployedTabuUAVS)) / float32(len(sol.deployedUavs))

		if tabuUavRatio > percentage {
			return problem.neighbourUavTabu(deviceId, uavId, configId, &possibleUavs, tabu, false)
		}
	}
	return problem.neighbourUav(deviceId, uavId, configId, &possibleUavs, false)
}

func (problem *UAVProblem) nextConfig(deviceId device.DeviceId, uavId, configId int32) uavConfigurationAssociation {
	devGwAss := deviceGatewayAssociation{deviceId, uavId}
	configIdx, found := slices.BinarySearch(problem.possibleConfigurations[devGwAss], configId)
	numPossibleConfigs := len(problem.possibleConfigurations[devGwAss])

	if found {
		// iterate config
		configIdx = int(math.Mod(float64(configIdx+1), float64(numPossibleConfigs)))
	} else {
		// the value found is already the next greater than current config
		configIdx = int(math.Mod(float64(configIdx), float64(numPossibleConfigs)))
	}

	configNextId := problem.possibleConfigurations[devGwAss][configIdx]

	return uavConfigurationAssociation{uavId, configNextId}
}

func (problem *UAVProblem) previousConfig(deviceId device.DeviceId, uavId, configId int32) uavConfigurationAssociation {
	devGwAss := deviceGatewayAssociation{deviceId, uavId}
	configIdx, _ := slices.BinarySearch(problem.possibleConfigurations[devGwAss], configId)
	numPossibleConfigs := len(problem.possibleConfigurations[devGwAss])

	configIdx = configIdx - 1
	if configIdx < 0 {
		configIdx = numPossibleConfigs - 1
	}

	configPrevId := problem.possibleConfigurations[devGwAss][configIdx]

	return uavConfigurationAssociation{uavId, configPrevId}
}

func (problem *UAVProblem) PrintCurrentSolution() {
	numDevices := problem.devices.Count()
	for key := 0; key < int(numDevices); key++ {
		fmt.Printf("Device %d -> %+v\n", key, problem.currentSolution.deviceAssociation[device.DeviceId(key)])
	}
}

func CreateUAVProblemInstance(alpha, beta, changeUav, newUavChance float64, devices *device.DeviceList, uavPositions *gateway.CandidatePositionList, gateway *gateway.Gateway) (*UAVProblem, error) {
	configs := make(map[int32]*device.Configuration, device.GetNumConfigurations())

	for sf := device.MinSF; sf <= device.MaxSF; sf++ {
		for tp := device.MinTP; tp <= device.MaxTP; tp += device.StepTP {
			configs[device.GetConfigID(sf, tp)] = &device.Configuration{
				Sf: int16(sf),
				Tp: int16(tp),
			}
		}
	}

	problem := &UAVProblem{
		gateway:        gateway,
		devices:        devices,
		uavPositions:   uavPositions,
		configurations: configs,
		alpha:          alpha,
		beta:           beta,
		changeUav:      changeUav,
		newUavChance:   newUavChance,
	}

	err := problem.processPossibleConfigurationPerDevice()
	if err != nil {
		return nil, err
	}

	return problem, nil
}
