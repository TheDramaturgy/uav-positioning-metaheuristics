package problem

import (
	"fmt"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/device"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/solver"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/utils"
	"math/rand"
	"slices"
)

const (
	debug bool = false
)

var globalIdx int64 = 0

type uavSliceKey struct {
	uavId int32
	slice int32
}

type UAVSolution struct {
	id                int64
	deviceAssociation map[device.DeviceId]*uavConfigurationAssociation
	uavSliceDevices   map[uavSliceKey][]device.DeviceId
	uavDevices        map[int32][]device.DeviceId
	uavDatarate       map[uavSliceKey]float32 // uavDatarate[gwID][sliceID] -> datarate
	deployedUavs      []int32
	problem           *UAVProblem
}

func (sol *UAVSolution) deployUav(uavId int32) {
	sol.deployedUavs = append(sol.deployedUavs, uavId)
}

func (sol *UAVSolution) withdrawUav(uavId int32) {
	slices.Sort(sol.deployedUavs)
	uavIdx, found := slices.BinarySearch(sol.deployedUavs, uavId)
	if found {
		utils.Pop(&sol.deployedUavs, uavIdx)
	}
}

func (sol *UAVSolution) GetDevicesAssignedTo(uavId int32) []device.DeviceId {
	return sol.uavDevices[uavId]
}

func (sol *UAVSolution) RemoveDeviceFromGateway(deviceId device.DeviceId, uavId int32) {
	// Update uavSliceDevices
	slice := sol.problem.devices.GetDevice(deviceId).Slice()
	key := uavSliceKey{uavId, slice}

	slices.Sort(sol.uavSliceDevices[key])
	deviceIdx, _ := slices.BinarySearch(sol.uavSliceDevices[key], deviceId)

	uavSliceDevices := sol.uavSliceDevices[key]
	utils.Pop(&uavSliceDevices, deviceIdx)
	sol.uavSliceDevices[key] = uavSliceDevices

	// Update uavDevices
	slices.Sort(sol.uavDevices[uavId])
	deviceIdx, _ = slices.BinarySearch(sol.uavDevices[uavId], deviceId)

	uavDevices := sol.uavDevices[uavId]
	utils.Pop(&uavDevices, deviceIdx)
	sol.uavDevices[uavId] = uavDevices

	// Last device removed from UAV
	if len(sol.uavDevices[uavId]) == 0 {
		sol.withdrawUav(uavId)
	}
}

func (sol *UAVSolution) AddDeviceToGateway(deviceId device.DeviceId, uavId int32) {
	// Update uavSliceDevices
	slice := sol.problem.devices.GetDevice(deviceId).Slice()
	key := uavSliceKey{uavId, slice}
	sol.uavSliceDevices[key] = append(sol.uavSliceDevices[key], deviceId)

	// Update uavDevices

	// First device added to UAV
	numDevicesInUav := len(sol.uavDevices[uavId])
	if numDevicesInUav == 0 {
		sol.deployUav(uavId)
	}

	if sol.uavDevices[uavId] == nil {
		sol.uavDevices[uavId] = make([]device.DeviceId, 0)
	}
	sol.uavDevices[uavId] = append(sol.uavDevices[uavId], deviceId)
}

func (sol *UAVSolution) GetAssignedUavId(deviceId device.DeviceId) int32 {
	return sol.deviceAssociation[deviceId].uavId
}

func (sol *UAVSolution) GetAssignedConfigId(deviceId device.DeviceId) int32 {
	return sol.deviceAssociation[deviceId].configId
}

func (sol *UAVSolution) Print() {
	numDevices := len(sol.deviceAssociation)
	for key := 0; key < int(numDevices); key++ {
		fmt.Printf("Device %d -> %+v\n", key, sol.deviceAssociation[device.DeviceId(key)])
	}
}

func (sol *UAVSolution) copy() *UAVSolution {
	copyDeviceAssociation := make(map[device.DeviceId]*uavConfigurationAssociation)
	copyUavSliceDevices := make(map[uavSliceKey][]device.DeviceId)
	copyUavDevices := make(map[int32][]device.DeviceId)
	copyUavDatarate := make(map[uavSliceKey]float32)

	for key, value := range sol.deviceAssociation {
		copyDeviceAssociation[key] = &uavConfigurationAssociation{value.uavId, value.configId}
	}

	for key, value := range sol.uavSliceDevices {
		copyUavSliceDevices[key] = make([]device.DeviceId, len(value))
		copy(copyUavSliceDevices[key], value)
	}

	for key, value := range sol.uavDevices {
		copyUavDevices[key] = make([]device.DeviceId, len(value))
		copy(copyUavDevices[key], value)
	}

	copyDeployedUavs := make([]int32, len(sol.deployedUavs))
	copy(copyDeployedUavs, sol.deployedUavs)

	for key, value := range sol.uavDatarate {
		copyUavDatarate[key] = value
	}

	id := globalIdx
	globalIdx++
	return &UAVSolution{
		id:                id,
		deviceAssociation: copyDeviceAssociation,
		uavSliceDevices:   copyUavSliceDevices,
		uavDevices:        copyUavDevices,
		uavDatarate:       copyUavDatarate,
		deployedUavs:      copyDeployedUavs,
		problem:           sol.problem,
	}
}

func (sol *UAVSolution) fixGatewayCapacity() {
	numUavPositions := sol.problem.uavPositions.Count()
	numSlices := int32(len(sol.problem.devices.Slices()))

	for uavId := int32(0); uavId < numUavPositions; uavId++ {
		for slice := int32(0); slice < numSlices; slice++ {
			key := uavSliceKey{uavId, slice}
			if sol.uavDatarate[key] > sol.problem.gateway.GetMaxDatarate(slice) {
				sol.unloadGateway(key)
			}
		}
	}
}

func (sol *UAVSolution) unloadGateway(key uavSliceKey) {
	devicesToMove := make([]device.DeviceId, len(sol.uavSliceDevices[key]))
	copy(devicesToMove, sol.uavSliceDevices[key])

	// Choose a random device to switch uav
	numDevices := int32(len(devicesToMove))
	if numDevices == 0 {
		panic("NO DEVICES")
	}
	deviceIdx := rand.Int31n(numDevices)
	deviceId := utils.Pop(&devicesToMove, int(deviceIdx))
	uavId := key.uavId
	configId := sol.GetAssignedConfigId(deviceId)

	currentDatarate := sol.uavDatarate[key]
	maxDatarate := sol.problem.gateway.GetMaxDatarate(key.slice)
	for currentDatarate > maxDatarate {
		// Get the next uav option
		association := sol.problem.nextUav(deviceId, uavId, configId)

		uavId = association.uavId

		// Check if we already verified all possible uavs
		if uavId != key.uavId {
			configId := association.configId
			sfNew := sol.problem.configurations[configId].Sf
			keyNew := uavSliceKey{uavId, key.slice}
			datarateNew := sol.problem.gateway.GetDatarate(sfNew, key.slice)

			if sol.uavDatarate[keyNew]+datarateNew <= sol.problem.gateway.GetMaxDatarate(key.slice) {
				// Current gateway is available. Assign device to it
				sol.updateDeviceAssociation(deviceId, &association)
			} else {
				// Current gateway is overloaded, try next
				currentDatarate = sol.uavDatarate[key]
				continue
			}
		}

		if len(devicesToMove) == 0 {
			// There is no way to move any device. Unfeasible Problem
			panic("impossible to unload gateway")
		}

		currentDatarate = sol.uavDatarate[key]

		// Try moving another random device
		numDevices = int32(len(devicesToMove))
		deviceIdx = rand.Int31n(numDevices)
		deviceId = utils.Pop(&devicesToMove, int(deviceIdx))
		uavId = key.uavId
	}
}

func (sol *UAVSolution) updateDeviceAssociation(deviceId device.DeviceId, association *uavConfigurationAssociation) {
	slice := sol.problem.devices.GetDevice(deviceId).Slice()
	if sol.deviceAssociation[deviceId] != nil {
		// Remove load from previous gateway
		configPrev := sol.GetAssignedConfigId(deviceId)
		uavPrev := sol.GetAssignedUavId(deviceId)
		keyPrev := uavSliceKey{uavPrev, slice}

		sfPrev := sol.problem.configurations[configPrev].Sf
		dataratePrev := sol.problem.gateway.GetDatarate(sfPrev, slice)
		sol.uavDatarate[keyPrev] -= dataratePrev

		// Remove device from previous gateway
		sol.RemoveDeviceFromGateway(deviceId, uavPrev)
	}

	// Add load to new gateway
	uavNew := association.uavId
	configNew := association.configId
	sfNew := sol.problem.configurations[configNew].Sf

	keyNew := uavSliceKey{uavNew, slice}
	datarateNew := sol.problem.gateway.GetDatarate(sfNew, slice)
	sol.uavDatarate[keyNew] += datarateNew

	// Update device association
	sol.deviceAssociation[deviceId] = association
	sol.AddDeviceToGateway(deviceId, uavNew)
}

func (sol *UAVSolution) neighbourUAV(deviceId device.DeviceId, uavId, configId int32) uavConfigurationAssociation {
	if debug {
		fmt.Println("-----------------------------------------------------------------------")
		fmt.Printf(" ChangingUAV -> ")
	}

	// Check probability of moving forward or backward
	random := utils.GetRandomProbability()
	if random <= 0.5 {
		if debug {
			fmt.Printf("Forward...\n")
		}
		return sol.problem.nextUav(deviceId, uavId, configId)
	} else {
		if debug {
			fmt.Printf("Backward...\n")
		}
		return sol.problem.previousUav(deviceId, uavId, configId)
	}
}

func (sol *UAVSolution) neighbourUAVSmarter(deviceId device.DeviceId, uavId, configId int32) uavConfigurationAssociation {
	if debug {
		fmt.Println("-----------------------------------------------------------------------")
		fmt.Printf(" ChangingUAV -> ")
	}

	/*
		Try next inside used_UAVS
		Get Random UnusedUAVs
	*/

	// Check probability of moving forward or backward
	random := utils.GetRandomProbability()
	if random <= 0.5 {
		if debug {
			fmt.Printf("Forward...\n")
		}
		random = utils.GetRandomProbability()
		if random < sol.problem.newUavChance {
			return sol.problem.nextNewUav(deviceId, uavId, configId, sol)
		} else {
			return sol.problem.nextDeployedUav(deviceId, uavId, configId, sol)
		}
	} else {
		if debug {
			fmt.Printf("Backward...\n")
		}
		random = utils.GetRandomProbability()
		if random < sol.problem.newUavChance {
			return sol.problem.previousNewUav(deviceId, uavId, configId, sol)
		} else {
			return sol.problem.previousDeployedUav(deviceId, uavId, configId, sol)
		}
	}
}

func (sol *UAVSolution) neighbourConfig(deviceId device.DeviceId, uavId, configId int32) uavConfigurationAssociation {
	if debug {
		fmt.Println("-----------------------------------------------------------------------")
		fmt.Printf(" ChangingConfig -> ")
	}

	// Check probability of moving forward or backward
	random := utils.GetRandomProbability()
	if random <= 0.5 {
		if debug {
			fmt.Printf("Forward...\n")
		}
		return sol.problem.nextConfig(deviceId, uavId, configId)
	} else {
		if debug {
			fmt.Printf("Backward...\n")
		}
		return sol.problem.previousConfig(deviceId, uavId, configId)
	}
}

func (sol *UAVSolution) GetNeighbour(minDistance, maxDistance int) solver.Solution {
	return sol.GetNeighbourSmarter(minDistance, maxDistance)
}

func (sol *UAVSolution) GetRandomNeighbour(minDistance, maxDistance int) *UAVSolution {
	distance := rand.Int31n(int32(maxDistance)-int32(minDistance)) + int32(minDistance)
	maxTies := 50

	neighbour := sol.copy()

	for i := int32(0); i < distance; i++ {
		deviceId := neighbour.problem.devices.GetRandomDevice().GetId()
		uavId := neighbour.GetAssignedUavId(deviceId)
		configId := neighbour.GetAssignedConfigId(deviceId)

		// Check probability of changing UAV or changing Configuration
		random := utils.GetRandomProbability()
		var ass uavConfigurationAssociation
		if random <= neighbour.problem.GetChanceOfChangingUAV() {
			ass = neighbour.neighbourUAV(deviceId, uavId, configId)
			if ass.uavId == uavId {
				// No available move. Try another one
				if maxTies > 0 {
					i--
					maxTies--
					continue
				}
			}
		} else {
			ass = neighbour.neighbourConfig(deviceId, uavId, configId)
			if ass.configId == configId {
				// No available move. Try another one
				if maxTies > 0 {
					i--
					maxTies--
					continue
				}
			}
		}

		if debug {
			fmt.Printf("Configs: %+v\n", sol.problem.possibleConfigurations[deviceGatewayAssociation{deviceId, uavId}])
			fmt.Printf("   Uavs: %+v\n", sol.problem.possibleUavs[deviceId])
			fmt.Printf(" Previous device: %d | uav: %d | config: %d \n", deviceId, uavId, configId)
			fmt.Printf("      New device: %d | uav: %d | config: %d \n", deviceId, ass.uavId, ass.configId)
			fmt.Println("-----------------------------------------------------------------------")
		}
		// Consolidate movement
		neighbour.updateDeviceAssociation(deviceId, &ass)
	}

	// Fix gateway capacity feasibility
	neighbour.fixGatewayCapacity()

	return neighbour
}

func (sol *UAVSolution) GetNeighbourSmarter(minDistance, maxDistance int) *UAVSolution {
	distance := rand.Int31n(int32(maxDistance)-int32(minDistance)) + int32(minDistance)
	maxTies := 50

	neighbour := sol.copy()

	for i := int32(0); i < distance; i++ {
		deviceId := neighbour.problem.devices.GetRandomDevice().GetId()
		uavId := neighbour.GetAssignedUavId(deviceId)
		configId := neighbour.GetAssignedConfigId(deviceId)

		// Check probability of changing UAV or changing Configuration
		random := utils.GetRandomProbability()
		var ass uavConfigurationAssociation
		if random <= neighbour.problem.GetChanceOfChangingUAV() {
			ass = neighbour.neighbourUAVSmarter(deviceId, uavId, configId)
			if ass.uavId == uavId {
				// No available move. Try another one
				if maxTies > 0 {
					i--
					maxTies--
					continue
				}
			}
		} else {
			ass = neighbour.neighbourConfig(deviceId, uavId, configId)
			if ass.configId == configId {
				// No available move. Try another one
				if maxTies > 0 {
					i--
					maxTies--
					continue
				}
			}
		}

		if debug {
			fmt.Printf("Configs: %+v\n", sol.problem.possibleConfigurations[deviceGatewayAssociation{deviceId, uavId}])
			fmt.Printf("   Uavs: %+v\n", sol.problem.possibleUavs[deviceId])
			fmt.Printf(" Previous device: %d | uav: %d | config: %d \n", deviceId, uavId, configId)
			fmt.Printf("      New device: %d | uav: %d | config: %d \n", deviceId, ass.uavId, ass.configId)
			fmt.Println("-----------------------------------------------------------------------")
		}
		// Consolidate movement
		neighbour.updateDeviceAssociation(deviceId, &ass)
	}

	// Fix gateway capacity feasibility
	neighbour.fixGatewayCapacity()

	return neighbour
}

func (sol *UAVSolution) GetCostA() float64 {
	uavs := make([]int32, 0)
	for _, association := range sol.deviceAssociation {
		uavs = append(uavs, association.uavId)
	}
	uniqueUavs := utils.Unique(&uavs)

	return float64(len(uniqueUavs)) * sol.problem.alpha
}

func (sol *UAVSolution) GetCostB() float64 {
	configs := map[int16]int32{7: 0, 8: 0, 9: 0, 10: 0, 11: 0, 12: 0}

	for _, association := range sol.deviceAssociation {
		configs[sol.problem.configurations[association.configId].Sf]++
	}

	maxSfCount := int32(0)
	for _, count := range configs {
		if count > maxSfCount {
			maxSfCount = count
		}
	}

	return float64(maxSfCount) * sol.problem.beta
}

func (sol *UAVSolution) GetCost() float64 {
	uavs := make([]int32, 0)
	configs := map[int16]int32{7: 0, 8: 0, 9: 0, 10: 0, 11: 0, 12: 0}

	for _, association := range sol.deviceAssociation {
		uavs = append(uavs, association.uavId)
		configs[sol.problem.configurations[association.configId].Sf]++
	}

	uniqueUavs := utils.Unique(&uavs)

	maxSfCount := int32(0)
	for _, count := range configs {
		if count > maxSfCount {
			maxSfCount = count
		}
	}

	return float64(len(uniqueUavs))*sol.problem.alpha + float64(maxSfCount)*sol.problem.beta
}

func GetRandomUAVSolution(problem *UAVProblem) (*UAVSolution, error) {
	numSlices := int32(len(problem.devices.Slices()))
	numUavs := problem.uavPositions.Count()
	numAssociations := numUavs * numSlices
	sol := &UAVSolution{
		deviceAssociation: make(map[device.DeviceId]*uavConfigurationAssociation, problem.devices.Count()),
		uavSliceDevices:   make(map[uavSliceKey][]device.DeviceId),
		uavDevices:        make(map[int32][]device.DeviceId),
		uavDatarate:       make(map[uavSliceKey]float32, numAssociations),
		deployedUavs:      make([]int32, 0),
		problem:           problem,
	}

	for uavId := int32(0); uavId < numUavs; uavId++ {
		for slice := int32(0); slice < numSlices; slice++ {
			key := uavSliceKey{uavId, slice}
			sol.uavSliceDevices[key] = make([]device.DeviceId, 0)
		}
	}

	// Random association for each device
	for deviceId := device.DeviceId(0); deviceId < device.DeviceId(problem.devices.Count()); deviceId++ {
		association := problem.getRandomUavConfiguration(deviceId)

		// Consolidate solution for device
		sol.updateDeviceAssociation(deviceId, &association)
	}

	// Check Gateway Capacities
	sol.fixGatewayCapacity()

	return sol, nil
}

func (sol *UAVSolution) OutputGatewayPositions() string {
	output := "id,x,y,z\n"
	uavs := make([]int32, 0)
	for _, association := range sol.deviceAssociation {
		uavs = append(uavs, association.uavId)
	}
	uniqueUavs := utils.Unique(&uavs)

	for _, uavId := range uniqueUavs {
		pos := sol.problem.uavPositions.GetCandidatePosition(uavId)
		output += fmt.Sprintf("%d,%f,%f,%f\n", uavId, pos.X, pos.Y, pos.Z)
	}

	return output
}

func (sol *UAVSolution) OutputConfigurations() string {
	output := "device,sf,tp\n"
	for key := device.DeviceId(0); key < device.DeviceId(sol.problem.devices.Count()); key++ {
		configId := sol.GetAssignedConfigId(key)
		config := sol.problem.configurations[configId]
		output += fmt.Sprintf("%d,%d,%d\n", key, config.Sf, config.Tp)
	}

	return output
}
