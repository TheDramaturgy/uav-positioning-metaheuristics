package utils

func Contains(list []int32, element int32) bool {
	for _, item := range list {
		if item == element {
			return true
		}
	}
	return false
}

func Pop[K any](s *[]K, i int) K {
	r := (*s)[i]
	*s = append((*s)[:i], (*s)[i+1:]...)
	return r
}

func Unique[T comparable](slice *[]T) []T {
	uniqueSlice := make([]T, 0, len(*slice))
	seen := make(map[T]bool, len(*slice))
	for _, el := range *slice {
		if !seen[el] {
			uniqueSlice = append(uniqueSlice, el)
			seen[el] = true
		}
	}

	return uniqueSlice
}

func Intersection[T comparable](slice1, slice2 []T) []T {
	inSlice1 := make(map[T]bool, len(slice1))
	for _, v := range slice1 {
		inSlice1[v] = true
	}

	intersect := []T{}
	for _, v := range slice2 {
		if _, ok := inSlice1[v]; ok {
			intersect = append(intersect, v)
		}
	}

	return intersect
}

func Complement[T comparable](slice, universe []T) []T {
	inSlice := make(map[T]bool, len(slice))
	for _, v := range slice {
		inSlice[v] = true
	}

	complement := []T{}
	for _, v := range universe {
		if _, ok := inSlice[v]; !ok {
			complement = append(complement, v)
		}
	}

	return complement
}
