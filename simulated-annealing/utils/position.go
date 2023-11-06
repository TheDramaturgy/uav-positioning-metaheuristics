package utils

import "math"

type Position struct {
	X float32
	Y float32
	Z float32
}

func (pos *Position) DistanceFrom(target Position) float32 {
	return float32(math.Sqrt(math.Pow(float64(pos.X)-float64(target.X), 2) + math.Pow(float64(pos.Y)-float64(target.Y), 2) + math.Pow(float64(pos.Z)-float64(target.Z), 2)))
}
