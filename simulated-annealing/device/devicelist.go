package device

import (
	"encoding/csv"
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/utils"
	"math/rand"
	"os"
	"strconv"
	"strings"
)

type DeviceList struct {
	count     int32
	devices   map[DeviceId]*Device
	deviceIds []DeviceId
	slices    []int32
}

func ReadDeviceList(devicePath, slicePath string) *DeviceList {
	deviceList := DeviceList{
		count:   0,
		devices: make(map[DeviceId]*Device, 0),
	}

	// Loading Device Positions

	file, err := os.Open(devicePath)
	if err != nil {
		panic(err)
	}

	reader := csv.NewReader(file)

	for {
		devicePos, err := reader.Read()

		if devicePos == nil {
			break
		}

		if err != nil {
			panic(err)
		}

		devicePos = strings.Split(devicePos[0], " ")
		devicePosX, err := strconv.ParseFloat(devicePos[0], 32)
		if err != nil {
			panic(err)
		}

		devicePosY, err := strconv.ParseFloat(devicePos[1], 32)
		if err != nil {
			panic(err)
		}

		devicePosZ, err := strconv.ParseFloat(devicePos[2], 32)
		if err != nil {
			panic(err)
		}
		dev := NewDevice(float32(devicePosX), float32(devicePosY), float32(devicePosZ))
		deviceList.addDevice(dev)
	}

	file.Close()

	// Loading Slice Associations

	file, err = os.Open(slicePath)
	if err != nil {
		panic(err)
	}
	defer file.Close()

	reader = csv.NewReader(file)

	for {
		association, err := reader.Read()

		if association == nil {
			break
		}

		if err != nil {
			panic(err)
		}

		association = strings.Split(association[0], " ")
		deviceId, err := strconv.ParseInt(association[0], 10, 32)
		if err != nil {
			panic(err)
		}

		slice, err := strconv.ParseInt(association[1], 10, 32)
		if err != nil {
			panic(err)
		}

		if !utils.Contains(deviceList.slices, int32(slice)) {
			deviceList.slices = append(deviceList.slices, int32(slice))
		}

		deviceList.devices[DeviceId(deviceId)].slice = int32(slice)
	}

	return &deviceList
}

func (dl *DeviceList) addDevice(device *Device) {
	device.id = DeviceId(dl.count)
	dl.devices[device.id] = device
	dl.deviceIds = append(dl.deviceIds, device.id)
	dl.count++
}

func (dl *DeviceList) GetDevice(deviceId DeviceId) *Device {
	return dl.devices[deviceId]
}

func (dl *DeviceList) Count() int32 {
	return dl.count
}

func (dl *DeviceList) GetRandomDevice() *Device {
	deviceIdx := rand.Int31n(int32(len(dl.deviceIds)))
	return dl.devices[dl.deviceIds[deviceIdx]]
}

func (dl *DeviceList) GetDeviceIds() []DeviceId {
	return dl.deviceIds
}

func (dl *DeviceList) Slices() []int32 {
	return dl.slices
}
