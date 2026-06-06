package customizer

import (
	"os"
	"runtime"
	"testing"

	"github.com/bytedance/gopkg/util/gopool"
)

func TestMain(m *testing.M) {
	gopool.SetCap(int32(runtime.NumCPU()))
	os.Exit(m.Run())
}
