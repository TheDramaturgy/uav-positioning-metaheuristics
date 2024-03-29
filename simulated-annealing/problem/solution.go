package problem

import (
	"cmp"
	"fmt"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/device"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/utils"
	"maps"
	"math/rand"
	"slices"
)

const (
	debug           bool = false
	DirectionUAV    int  = 1
	DirectionConfig int  = 2
)

var globalIdx int64 = 0

type Solution interface {
	GetInverseCost() float64
	GetCost() float64
	GetCostA() float64
	GetCostB() float64
	GetDeployedUavs() []int32
	GetNeighbourSA(minDistance, maxDistance int) Solution
	GetNeighbourRandom(minDistance, maxDistance int) Solution
	GetNeighbourList(size int) []Solution
	GetGeneratingMove() Move
	GetAssociations() []Association
	String() string
	Copy() Solution
	FlipAssociation(association Association)
	GetUavTabuRatio(tabu []int32) float32
}

type Move struct {
	DeviceId   device.DeviceId
	Direction  int
	PrevConfig int32
	PrevUAV    int32
	NewConfig  int32
	NewUAV     int32
}

type uavSliceKey struct {
	uavId int32
	slice int32
}

type Association struct {
	Device device.DeviceId
	Uav    int32
	Config int32
}

type UAVSolution struct {
	id                int64
	deviceAssociation map[device.DeviceId]uavConfigurationAssociation
	uavSliceDevices   map[uavSliceKey][]device.DeviceId
	uavDevices        map[int32][]device.DeviceId
	uavDatarate       map[uavSliceKey]float32 // uavDatarate[gwID][sliceID] -> datarate
	deployedUavs      []int32
	generatingMove    Move
	cost              float64
	problem           *UAVProblem
}

func (sol *UAVSolution) GetAssociations() []Association {
	associations := make([]Association, 0)
	for key, value := range sol.deviceAssociation {
		associations = append(associations, Association{key, value.uavId, value.configId})
	}

	return associations
}

func (sol *UAVSolution) String() string {
	t := fmt.Sprintf("id: %d, cost: %f, move: %+v\n", sol.id, sol.GetCost(), sol.generatingMove)
	for i, _ := range sol.problem.devices.GetDeviceIds() {
		t += fmt.Sprintf(" (%d,%d),", sol.deviceAssociation[device.DeviceId(i)].uavId, sol.deviceAssociation[device.DeviceId(i)].configId)
	}
	return t
}

func (sol *UAVSolution) GetGeneratingMove() Move {
	return sol.generatingMove
}

func (sol *UAVSolution) GetUavTabuRatio(tabu []int32) float32 {
	deployedTabuUAVS := utils.Intersection(sol.deployedUavs, tabu)
	return float32(len(deployedTabuUAVS)) / float32(len(sol.deployedUavs))
}

func (sol *UAVSolution) GetDeployedUavs() []int32 {
	return slices.Clone(sol.deployedUavs)
}

func (sol *UAVSolution) GetDeployedUavsGene() []bool {
	uavs := make([]bool, sol.problem.uavPositions.Count())
	for _, uavId := range sol.deployedUavs {
		uavs[uavId] = true
	}
	return uavs
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
	for key := 0; key < numDevices; key++ {
		fmt.Printf("Device %d -> %+v\n", key, sol.deviceAssociation[device.DeviceId(key)])
	}
}

func (sol *UAVSolution) copy() *UAVSolution {
	copyUavSliceDevices := make(map[uavSliceKey][]device.DeviceId)
	copyUavDevices := make(map[int32][]device.DeviceId)
	//copyUavDatarate := make(map[uavSliceKey]float32)

	copyDeviceAssociation := maps.Clone(sol.deviceAssociation)
	copyUavDatarate := maps.Clone(sol.uavDatarate)

	//for key, value := range sol.deviceAssociation {
	//	copyDeviceAssociation[key] = &uavConfigurationAssociation{value.uavId, value.configId}
	//}
	//
	for key, value := range sol.uavSliceDevices {
		copyUavSliceDevices[key] = slices.Clone(value)
	}

	for key, value := range sol.uavDevices {
		copyUavDevices[key] = slices.Clone(value)
	}

	//copyDeployedUavs := make([]int32, len(sol.deployedUavs))
	//copy(copyDeployedUavs, sol.deployedUavs)

	copyDeployedUavs := slices.Clone(sol.deployedUavs)

	//for key, value := range sol.uavDatarate {
	//	copyUavDatarate[key] = value
	//}

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

func (sol *UAVSolution) Copy() Solution {
	return sol.copy()
}

func (sol *UAVSolution) fixGatewayCapacity() bool {
	numUavPositions := sol.problem.uavPositions.Count()
	numSlices := int32(len(sol.problem.devices.Slices()))
	for uavId := int32(0); uavId < numUavPositions; uavId++ {
		for slice := int32(0); slice < numSlices; slice++ {
			key := uavSliceKey{uavId, slice}
			if sol.uavDatarate[key] > sol.problem.gateway.GetMaxDatarate(slice) {
				//fmt.Printf("(!!!) Fixing gateway capacity...\n")
				if !sol.unloadGateway(key) {
					panic("Impossible to fix Gateway")
				}
			}
		}
	}

	return true
}

func (sol *UAVSolution) IsFeasible() bool {
	numUavPositions := sol.problem.uavPositions.Count()
	numSlices := int32(len(sol.problem.devices.Slices()))
	for uavId := int32(0); uavId < numUavPositions; uavId++ {
		for slice := int32(0); slice < numSlices; slice++ {
			key := uavSliceKey{uavId, slice}
			if sol.uavDatarate[key] > sol.problem.gateway.GetMaxDatarate(slice) {
				return false
			}
		}
	}

	return true
}

func (sol *UAVSolution) unloadGateway(key uavSliceKey) bool {
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
				sol.updateDeviceAssociation(deviceId, association)
			} else {
				// Current gateway is overloaded, try next
				currentDatarate = sol.uavDatarate[key]
				continue
			}
		}

		if len(devicesToMove) == 0 {
			// There is no way to move any device. Unfeasible Problem
			return false
		}

		currentDatarate = sol.uavDatarate[key]

		// Try moving another random device
		numDevices = int32(len(devicesToMove))
		deviceIdx = rand.Int31n(numDevices)
		deviceId = utils.Pop(&devicesToMove, int(deviceIdx))
		uavId = key.uavId
	}

	return true
}

func (sol *UAVSolution) updateDeviceAssociation(deviceId device.DeviceId, association uavConfigurationAssociation) {
	slice := sol.problem.devices.GetDevice(deviceId).Slice()
	if sol.deviceAssociation[deviceId] != *(&uavConfigurationAssociation{}) {
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

func (sol *UAVSolution) FlipAssociation(association Association) {
	deviceId := association.Device
	uavId := association.Uav
	configId := association.Config

	if sol.deviceAssociation[deviceId].uavId != uavId || sol.deviceAssociation[deviceId].configId != configId {
		sol.updateDeviceAssociation(deviceId, uavConfigurationAssociation{uavId: uavId, configId: configId})
		return
	}

	ass := sol.problem.getRandomUavConfiguration(deviceId)
	sol.updateDeviceAssociation(deviceId, ass)
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
			return sol.problem.nextNewUav(deviceId, uavId, configId, sol, []int32{}, 1.0)
		} else {
			return sol.problem.nextDeployedUav(deviceId, uavId, configId, sol, []int32{})
		}
	} else {
		if debug {
			fmt.Printf("Backward...\n")
		}
		random = utils.GetRandomProbability()
		if random < sol.problem.newUavChance {
			return sol.problem.previousNewUav(deviceId, uavId, configId, sol, []int32{}, 1.0)
		} else {
			return sol.problem.previousDeployedUav(deviceId, uavId, configId, sol, []int32{})
		}
	}
}

func (sol *UAVSolution) neighbourUAVSmarterTabu(deviceId device.DeviceId, uavId, configId int32, uavTabu []int32, tabuPercentage float32) uavConfigurationAssociation {
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
			return sol.problem.nextNewUav(deviceId, uavId, configId, sol, uavTabu, tabuPercentage)
		} else {
			return sol.problem.nextDeployedUav(deviceId, uavId, configId, sol, uavTabu)
		}
	} else {
		if debug {
			fmt.Printf("Backward...\n")
		}
		random = utils.GetRandomProbability()
		if random < sol.problem.newUavChance {
			return sol.problem.previousNewUav(deviceId, uavId, configId, sol, uavTabu, tabuPercentage)
		} else {
			return sol.problem.previousDeployedUav(deviceId, uavId, configId, sol, uavTabu)
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

func (sol *UAVSolution) GetNeighbourSA(minDistance, maxDistance int) Solution {
	distance := rand.Int31n(int32(maxDistance)-int32(minDistance)) + int32(minDistance)
	newSol := sol.copy()
	for i := int32(0); i < distance; i++ {
		newSol = newSol.GetNeighbourSmarter()
	}
	return newSol
}

func (sol *UAVSolution) GetNeighbourRandom(minDistance, maxDistance int) Solution {
	distance := rand.Int31n(int32(maxDistance)-int32(minDistance)) + int32(minDistance)
	newSol := sol.copy()
	for i := int32(0); i < distance; i++ {
		newSol = newSol.getNeighbourRandom()
	}
	return newSol
}

func (sol *UAVSolution) GetNeighbourList(size int) []Solution {
	solutions := make([]Solution, size)

	for i := 0; i < size; i++ {
		solutions[i] = sol.GetNeighbourSmarter()
	}

	return solutions
}

func (sol *UAVSolution) GetNeighbourListTabu(size int, uavTabu []int32, tabuPercentage float32) []Solution {
	solutions := make([]Solution, size)

	for i := 0; i < size; i++ {
		solutions[i] = sol.GetNeighbourSmarter()
	}

	return solutions
}

func (sol *UAVSolution) getNeighbourRandom() *UAVSolution {
	maxTies := 50

	neighbour := sol.copy()
	var move Move

	for maxTies > 0 {
		deviceId := neighbour.problem.devices.GetRandomDevice().GetId()
		uavId := neighbour.GetAssignedUavId(deviceId)
		configId := neighbour.GetAssignedConfigId(deviceId)

		move.DeviceId = deviceId

		// Check probability of changing UAV or changing Configuration
		random := utils.GetRandomProbability()
		var ass uavConfigurationAssociation
		if random <= neighbour.problem.GetChanceOfChangingUAV() {
			move.Direction = DirectionUAV
			ass = neighbour.neighbourUAV(deviceId, uavId, configId)
			if ass.uavId == uavId {
				// No available move. Try another one
				if maxTies > 0 {
					maxTies--
					continue
				}
			}
		} else {
			move.Direction = DirectionConfig
			ass = neighbour.neighbourConfig(deviceId, uavId, configId)
			if ass.configId == configId {
				// No available move. Try another one
				if maxTies > 0 {
					maxTies--
					continue
				}
			}
		}

		move.PrevUAV = uavId
		move.PrevConfig = configId
		move.NewUAV = ass.uavId
		move.NewConfig = ass.configId

		if debug {
			fmt.Printf("Configs: %+v\n", sol.problem.possibleConfigurations[deviceGatewayAssociation{deviceId, uavId}])
			fmt.Printf("   Uavs: %+v\n", sol.problem.possibleUavs[deviceId])
			fmt.Printf(" Previous device: %d | uav: %d | config: %d \n", deviceId, uavId, configId)
			fmt.Printf("      New device: %d | uav: %d | config: %d \n", deviceId, ass.uavId, ass.configId)
			fmt.Println("-----------------------------------------------------------------------")
		}

		// Consolidate movement
		neighbour.updateDeviceAssociation(deviceId, ass)
		break
	}

	if maxTies == 0 {
		panic("No valid movement found")
	}

	// Fix gateway capacity feasibility
	neighbour.fixGatewayCapacity()
	neighbour.generatingMove = move

	return neighbour
}

func (sol *UAVSolution) GetNeighbourSmarter() *UAVSolution {
	maxTies := 50

	neighbour := sol.copy()
	var move Move

	for maxTies > 0 {
		deviceId := neighbour.problem.devices.GetRandomDevice().GetId()
		uavId := neighbour.GetAssignedUavId(deviceId)
		configId := neighbour.GetAssignedConfigId(deviceId)

		move.DeviceId = deviceId

		// Check probability of changing UAV or changing Configuration
		random := utils.GetRandomProbability()
		var ass uavConfigurationAssociation
		if random <= neighbour.problem.GetChanceOfChangingUAV() {
			move.Direction = DirectionUAV
			ass = neighbour.neighbourUAVSmarter(deviceId, uavId, configId)
			if ass.uavId == uavId {
				// No available move. Try another one
				if maxTies > 0 {
					maxTies--
					continue
				}
			}
		} else {
			move.Direction = DirectionConfig
			ass = neighbour.neighbourConfig(deviceId, uavId, configId)
			if ass.configId == configId {
				// No available move. Try another one
				if maxTies > 0 {
					maxTies--
					continue
				}
			}

		}

		move.PrevUAV = uavId
		move.PrevConfig = configId
		move.NewUAV = ass.uavId
		move.NewConfig = ass.configId

		if debug {
			fmt.Printf("Configs: %+v\n", sol.problem.possibleConfigurations[deviceGatewayAssociation{deviceId, uavId}])
			fmt.Printf("   Uavs: %+v\n", sol.problem.possibleUavs[deviceId])
			fmt.Printf(" Previous device: %d | uav: %d | config: %d \n", deviceId, uavId, configId)
			fmt.Printf("      New device: %d | uav: %d | config: %d \n", deviceId, ass.uavId, ass.configId)
			fmt.Println("-----------------------------------------------------------------------")
		}

		// Consolidate movement
		neighbour.updateDeviceAssociation(deviceId, ass)
		break
	}

	if maxTies == 0 {
		fmt.Errorf("No valid movement found\n")
	}

	// Fix gateway capacity feasibility
	neighbour.fixGatewayCapacity()
	neighbour.generatingMove = move

	return neighbour
}

func (sol *UAVSolution) GetNeighbourSmarterTabu(uavTabu []int32, tabuPercentage float32) *UAVSolution {
	maxTies := 50

	neighbour := sol.copy()
	var move Move

	tabuDevices := make([]device.DeviceId, 0)
	tabuUavRatio := float32(0.0)
	if len(uavTabu) > 0 {
		for _, deviceId := range sol.problem.GetDeviceIds() {
			found := utils.Contains(uavTabu, sol.deviceAssociation[deviceId].uavId)
			if found {
				tabuDevices = append(tabuDevices, deviceId)
			}
		}

		deployedTabuUAVS := utils.Intersection(sol.deployedUavs, uavTabu)
		tabuUavRatio = float32(len(deployedTabuUAVS)) / float32(len(sol.deployedUavs))
	}

	for maxTies > 0 {
		deviceId := neighbour.problem.devices.GetRandomDevice().GetId()
		if len(tabuDevices) > 0 && tabuUavRatio > tabuPercentage {
			random := utils.GetRandomProbability()
			if random <= 0.75 {
				deviceId = tabuDevices[rand.Int31n(int32(len(tabuDevices)))]
			}
		}
		uavId := neighbour.GetAssignedUavId(deviceId)
		configId := neighbour.GetAssignedConfigId(deviceId)

		move.DeviceId = deviceId

		// Check probability of changing UAV or changing Configuration
		random := utils.GetRandomProbability()
		var ass uavConfigurationAssociation
		if random <= neighbour.problem.GetChanceOfChangingUAV() {
			move.Direction = DirectionUAV
			ass = neighbour.neighbourUAVSmarterTabu(deviceId, uavId, configId, uavTabu, tabuPercentage)
			if ass.uavId == uavId {
				// No available move. Try another one
				if maxTies > 0 {
					maxTies--
					continue
				}
			}
		} else {
			move.Direction = DirectionConfig
			ass = neighbour.neighbourConfig(deviceId, uavId, configId)
			if ass.configId == configId {
				// No available move. Try another one
				if maxTies > 0 {
					maxTies--
					continue
				}
			}

		}

		move.PrevUAV = uavId
		move.PrevConfig = configId
		move.NewUAV = ass.uavId
		move.NewConfig = ass.configId

		if debug {
			fmt.Printf("Configs: %+v\n", sol.problem.possibleConfigurations[deviceGatewayAssociation{deviceId, uavId}])
			fmt.Printf("   Uavs: %+v\n", sol.problem.possibleUavs[deviceId])
			fmt.Printf(" Previous device: %d | uav: %d | config: %d \n", deviceId, uavId, configId)
			fmt.Printf("      New device: %d | uav: %d | config: %d \n", deviceId, ass.uavId, ass.configId)
			fmt.Println("-----------------------------------------------------------------------")
		}

		// Consolidate movement
		neighbour.updateDeviceAssociation(deviceId, ass)
		break
	}

	if maxTies == 0 {
		panic("No valid movement found")
	}

	// Fix gateway capacity feasibility
	neighbour.fixGatewayCapacity()
	neighbour.generatingMove = move

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
	if sol.cost == 0 {
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

		sol.cost = float64(len(uniqueUavs))*sol.problem.alpha + float64(maxSfCount)*sol.problem.beta
	}

	return sol.cost
}

func (sol *UAVSolution) GetInverseCost() float64 {
	cost := sol.GetCost()
	maxCost := float64(len(sol.problem.devices.GetDeviceIds())) * (sol.problem.alpha + sol.problem.beta)
	return maxCost - cost
}

func GetRandomUAVSolution(problem *UAVProblem) (*UAVSolution, error) {
	numSlices := int32(len(problem.devices.Slices()))
	numUavs := problem.uavPositions.Count()
	numAssociations := numUavs * numSlices
	id := globalIdx
	globalIdx++
	sol := &UAVSolution{
		id:                id,
		deviceAssociation: make(map[device.DeviceId]uavConfigurationAssociation, problem.devices.Count()),
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
		sol.updateDeviceAssociation(deviceId, association)
	}

	// Check Gateway Capacities
	sol.fixGatewayCapacity()

	return sol, nil
}

func GetUAVSolution(problem *UAVProblem, uavs []int32, coverage map[int32][]device.DeviceId, defaultSF int16) (*UAVSolution, error) {
	numSlices := int32(len(problem.devices.Slices()))
	numUavs := problem.uavPositions.Count()
	numAssociations := numUavs * numSlices
	sol := &UAVSolution{
		deviceAssociation: make(map[device.DeviceId]uavConfigurationAssociation, problem.devices.Count()),
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

	for _, uavId := range uavs {
		for _, deviceId := range coverage[uavId] {
			association := problem.getConfigurationForUAV(deviceId, uavId, defaultSF)

			// Consolidate solution for device
			sol.updateDeviceAssociation(deviceId, association)
		}
	}

	sol.fixGatewayCapacity()
	return sol, nil
}

func GetUAVSolutionFromDeployedUAVs(problem *UAVProblem, uavs []int32) (*UAVSolution, error) {
	deviceCoverage := make(map[device.DeviceId]map[int][]int32)

	for _, dev := range problem.GetDeviceIds() {
		deviceCoverage[dev] = make(map[int][]int32)
		for i := device.MinSF; i <= device.MaxSF; i++ {
			deviceCoverage[dev][i] = make([]int32, 0)
		}

		for _, uav := range uavs {
			for _, sf := range problem.GetPossibleSFs(dev, uav) {
				deviceCoverage[dev][sf] = append(deviceCoverage[dev][sf], uav)
			}
		}
	}

	uncoveredDevices := problem.GetDeviceIds()

	slices.SortFunc(uncoveredDevices, func(i, j device.DeviceId) int {
		sf := 10
		return cmp.Compare(len(deviceCoverage[i][sf]), len(deviceCoverage[j][sf]))
	})

	//slices.SortFunc(uncoveredDevices, func(i, j device.DeviceId) int {
	//	for sf := device.MinSF; sf <= device.MaxSF; sf++ {
	//		if len(deviceCoverage[i][sf]) == 0 && len(deviceCoverage[j][sf]) == 0 {
	//			continue
	//		}
	//
	//		if len(deviceCoverage[j][sf]) == 0 {
	//			return -1
	//		}
	//
	//		if len(deviceCoverage[i][sf]) == 0 {
	//			return 1
	//		}
	//
	//		return cmp.Compare(len(deviceCoverage[i][sf]), len(deviceCoverage[j][sf]))
	//	}
	//
	//	return 0
	//})

	numSlices := int32(len(problem.devices.Slices()))
	numUavs := problem.uavPositions.Count()
	numAssociations := numUavs * numSlices
	sol := &UAVSolution{
		deviceAssociation: make(map[device.DeviceId]uavConfigurationAssociation, problem.devices.Count()),
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

	sf := 7
	uavDevices := make(map[int32]int, len(uavs))
	for idx, devId := range uncoveredDevices {
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// USING SF10 FOR ALL DEVICES
		sf = 10

		//for len(deviceCoverage[devId][sf]) == 0 {
		//	sf++
		//	if sf > device.MaxSF {
		//		newUav := getFixUAV(devId, uavs, problem)
		//		uavs = append(uavs, newUav)
		//		for i := idx; i < len(uncoveredDevices); i++ {
		//			dev := uncoveredDevices[i]
		//			for _, sf := range problem.GetPossibleSFs(dev, newUav) {
		//				deviceCoverage[dev][sf] = append(deviceCoverage[dev][sf], newUav)
		//			}
		//		}
		//
		//		sf = 7
		//		//panic("Infeasible")
		//	}
		//}

		slice := problem.GetSlice(devId)
		selectedUav := int32(-1)
		cover := deviceCoverage[devId][sf]
		for _, uavId := range cover {
			key := uavSliceKey{uavId, slice}
			dr := sol.problem.gateway.GetDatarate(int16(sf), slice)
			if sol.uavDatarate[key]+dr > sol.problem.gateway.GetMaxDatarate(slice) {
				continue
			}

			if selectedUav == -1 || uavDevices[uavId] < uavDevices[selectedUav] {
				selectedUav = uavId
			}
		}

		if selectedUav == -1 {
			newUav := getFixUAV(devId, uavs, problem)
			uavs = append(uavs, newUav)
			for i := idx; i < len(uncoveredDevices); i++ {
				dev := uncoveredDevices[i]
				for _, sf := range problem.GetPossibleSFs(dev, newUav) {
					deviceCoverage[dev][sf] = append(deviceCoverage[dev][sf], newUav)
				}
			}

			selectedUav = newUav
			//panic("Infeasible")
		}

		association := problem.getConfigurationForUAV(devId, selectedUav, int16(sf))

		// Consolidate solution for device
		sol.updateDeviceAssociation(devId, association)
	}

	sol.fixGatewayCapacity()
	return sol, nil
}

func getFixUAV(devId device.DeviceId, deployedUavs []int32, instance *UAVProblem) int32 {
	uavs := instance.GetUAVIds()
	coverage := make(map[int32][]device.DeviceId, len(instance.GetUAVIds()))
	for _, uavId := range uavs {
		coverage[uavId] = instance.GetCoverage(uavId)
	}

	slices.SortFunc(uavs, func(i, j int32) int {
		return cmp.Compare(len(coverage[j]), len(coverage[i]))
	})

	selected := int32(-1)
	for _, i := range uavs {
		if slices.Contains(deployedUavs, i) {
			continue
		}

		if slices.Contains(coverage[i], devId) {
			selected = i
			break
		}
	}

	if selected == -1 {
		panic("Infeasible")
	}

	return selected
}

func CrossoverOld(sol1, sol2 *UAVSolution, cprob, mprob float64) (*UAVSolution, *UAVSolution) {
	sol1Copy := sol1.copy()
	sol2Copy := sol2.copy()

	numDevices := int32(len(sol1.problem.devices.GetDeviceIds()))
	pivotId := device.DeviceId(rand.Int31n(numDevices - 1))

	r := rand.Float64()
	if r > cprob {
		pivotId = device.DeviceId(numDevices)
	}

	for id := device.DeviceId(0); int32(id) < numDevices; id++ {
		if id >= pivotId {
			uavId1 := sol1Copy.GetAssignedUavId(id)
			configId1 := sol1Copy.GetAssignedConfigId(id)
			uavId2 := sol2Copy.GetAssignedUavId(id)
			configId2 := sol2Copy.GetAssignedConfigId(id)

			sol1Copy.updateDeviceAssociation(id, uavConfigurationAssociation{uavId2, configId2})
			sol2Copy.updateDeviceAssociation(id, uavConfigurationAssociation{uavId1, configId1})
		}

		if sol1Copy.mutate(id, mprob) {
			//fmt.Printf("--------------------- C1 Mutated ---------------------\n")
		}
		if sol2Copy.mutate(id, mprob) {
			//fmt.Printf("--------------------- C2 Mutated ---------------------\n")
		}
	}

	return sol1Copy, sol2Copy
}

func Crossover(sol1, sol2 *UAVSolution, cprob, mprob float64) (*UAVSolution, *UAVSolution) {
	sol1UAVs := sol1.GetDeployedUavsGene()
	sol2UAVs := sol2.GetDeployedUavsGene()

	uavIDs := sol1.problem.GetUAVIds()
	numUAVs := int32(len(uavIDs))
	pivotUAV := int32(rand.Int31n(numUAVs - 1))

	r := rand.Float64()
	if r > cprob {
		pivotUAV = numUAVs
	}

	for uav := int32(0); uav < numUAVs; uav++ {
		if uav >= pivotUAV {
			sol1UAVs[uav] = sol2UAVs[uav]
			sol2UAVs[uav] = sol1UAVs[uav]
		}

		if shouldMutate(mprob) {
			//fmt.Printf("--------------------- C1 Mutated ---------------------\n")
			sol1UAVs[uav] = !sol1UAVs[uav]
		}

		if shouldMutate(mprob) {
			//fmt.Printf("--------------------- C2 Mutated ---------------------\n")
			sol2UAVs[uav] = !sol2UAVs[uav]
		}
	}

	deployedUAVs1 := make([]int32, 0)
	for uavId, deployed := range sol1UAVs {
		if deployed {
			deployedUAVs1 = append(deployedUAVs1, int32(uavId))
		}
	}

	deployedUAVs2 := make([]int32, 0)
	for uavId, deployed := range sol2UAVs {
		if deployed {
			deployedUAVs2 = append(deployedUAVs2, int32(uavId))
		}
	}

	child1, _ := GetUAVSolutionFromDeployedUAVs(sol1.problem, deployedUAVs1)
	child2, _ := GetUAVSolutionFromDeployedUAVs(sol2.problem, deployedUAVs2)

	return child1, child2
}

func GetRandomUAVSolutionTabu(problem *UAVProblem, tabuUavs []int32, tabuRatio float32) (*UAVSolution, error) {
	numSlices := int32(len(problem.devices.Slices()))
	numUavs := problem.uavPositions.Count()
	numAssociations := numUavs * numSlices
	sol := &UAVSolution{
		deviceAssociation: make(map[device.DeviceId]uavConfigurationAssociation, problem.devices.Count()),
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

	usedUavs := make([]int32, 0)
	// Random association for each device
	for deviceId := device.DeviceId(0); deviceId < device.DeviceId(problem.devices.Count()); deviceId++ {
		deployedTabuUAVS := utils.Intersection(usedUavs, tabuUavs)
		currTabuUavRatio := float32(len(deployedTabuUAVS)) / float32(len(tabuUavs))
		association := problem.getRandomUavConfigurationTabu(deviceId, usedUavs, tabuUavs, tabuRatio, currTabuUavRatio)

		usedUavs = append(usedUavs, association.uavId)

		// Consolidate solution for device
		sol.updateDeviceAssociation(deviceId, association)
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

func (sol *UAVSolution) mutate(id device.DeviceId, mprob float64) bool {
	r := rand.Float64()
	if r < mprob {
		sol.updateDeviceAssociation(id, sol.problem.getRandomUavConfiguration(id))
		return true
	}
	return false
}

func shouldMutate(mprob float64) bool {
	r := rand.Float64()
	if r < mprob {
		return true
	}
	return false
}
