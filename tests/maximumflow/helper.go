package maximumflow

import (
	"bufio"
	"errors"
	"io"
	"strings"

	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
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

func flattenEdges(es [][]pairEdge) []osmparser.Edge {
	flatten := make([]osmparser.Edge, 0, len(es))

	eid := 0

	for from, edges := range es {
		for _, e := range edges {
			flatten = append(flatten, osmparser.NewEdge(uint32(from), uint32(e.to), e.weight, 0, uint32(eid), 0))
			eid++
		}
	}

	return flatten
}
