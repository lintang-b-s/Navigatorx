// Package maximumflow contains tests and helpers for maximum flow algorithms.
package maximumflow

import (
	"bufio"
	"errors"
	"io"
	"strings"
)

func readLine(br *bufio.Reader) (string, error) {
	line, err := br.ReadString('\n')
	if err != nil {
		if errors.Is(err, io.EOF) && len(line) > 0 {
		} else {
			return "", err
		}
	}
	return strings.TrimRight(line, "\r\n"), nil
}

func fields(s string) []string {

	return strings.Fields(s)
}

type pairEdge struct {
	to     int
	weight float64
}

func newPairEdge(to int, weight float64) pairEdge {
	return pairEdge{to, weight}
}
