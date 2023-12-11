package device

import (
	"github.com/TheDramaturgy/uav-positioning-metaheuristics/simulated-annealing/utils"
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

func (d *Device) Copy() *Device {
	return &Device{
		id:    d.id,
		pos:   utils.Position{d.pos.X, d.pos.Y, d.pos.Z},
		slice: d.slice,
	}
}
