package device

import (
	"simulated-annealing/utils"
)

type DeviceId int32

type Device struct {
	id    DeviceId
	pos   utils.Position
	slice int32
}

func NewDevice(x, y, z float32) *Device {
	return &Device{
		pos: utils.Position{x, y, z},
	}
}

func (d *Device) GetId() DeviceId {
	return d.id
}

func (d *Device) GetPosition() utils.Position {
	return d.pos
}

func (d *Device) Slice() int32 {
	return d.slice
}
