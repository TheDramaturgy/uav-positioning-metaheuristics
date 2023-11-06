package main

import (
	"fmt"
	"strings"
	"testing"
)

func BenchmarkConcat(b *testing.B) { // 58.5
	for i := 0; i < b.N; i++ {
		var s string = ""
		for j := 0; j < 100000; j++ {
			s += fmt.Sprintf("%d,%f\n", j, (0.4 * float64(j)))
		}
	}
}

func BenchmarkBuilder(b *testing.B) { // 58.5
	for i := 0; i < b.N; i++ {
		var s strings.Builder
		s.Grow(32)
		for j := 0; j < 100000; j++ {
			s.WriteString(fmt.Sprintf("%d,%f\n", j, (0.4 * float64(j))))
		}
		_ = s.String()
	}
}

func BenchmarkBuilderWithAlloc(b *testing.B) { // 58.5
	for i := 0; i < b.N; i++ {
		var s strings.Builder
		s.Grow(32 * 100000)
		for j := 0; j < 100000; j++ {
			s.WriteString(fmt.Sprintf("%d,%f\n", j, (0.4 * float64(j))))
		}
		_ = s.String()
	}
}
