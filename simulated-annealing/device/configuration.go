package device

const (
	MinSF  = 7
	MaxSF  = 12
	MinTP  = 2
	MaxTP  = 14
	StepTP = 2
)

type Configuration struct {
	Sf int16
	Tp int16
}

func GetNumConfigurations() int32 {
	return GetConfigID(MaxSF, MaxTP) + 1
}

// ConfigId = (sf - 7) * 7 + (tp/2 - 1)
func GetConfigID(sf, tp int) int32 {
	return int32((sf-MinSF)*(MaxTP/StepTP) + (tp/StepTP - 1))
}
