package gateway

import "math"

type Gateway struct {
	bandwidths   map[int32]float32
	maxDatarates map[int32]float32
	sensitivity  map[int16]float32
	slices       []int32
}

func (gateway *Gateway) AddSlice(sliceId int32, bandwidth, maxDatarate float32) {
	if gateway.bandwidths == nil {
		gateway.bandwidths = make(map[int32]float32)
		gateway.maxDatarates = make(map[int32]float32)
	}
	gateway.slices = append(gateway.slices, sliceId)
	gateway.bandwidths[sliceId] = bandwidth
	gateway.maxDatarates[sliceId] = maxDatarate
}

func (gateway *Gateway) GetBandwidth(sliceId int32) float32 {
	return gateway.bandwidths[sliceId]
}

func (gateway *Gateway) GetMaxDatarate(sliceId int32) float32 {
	return gateway.maxDatarates[sliceId]
}

func (gateway *Gateway) SetSensitivity(sensitivity map[int16]float32) {
	gateway.sensitivity = sensitivity
}

func (gateway *Gateway) GetSensitivityForSf(sf int16) float32 {
	return gateway.sensitivity[sf]
}

func (gateway *Gateway) GetDatarate(sf int16, sliceId int32) float32 {
	return float32(sf) * gateway.bandwidths[sliceId] / float32(math.Pow(2, float64(sf)))
}
